// tts_stream.cpp - 流水线 TTS：ZMQ 接收文本 + 分句 + 合成线程 + 播放线程
//
// 架构:
//   ZMQ SUB → 分句器 → 句子队列 → 合成线程 → 音频队列 → 播放线程 → 扬声器
//
// 用法:
//   ./tts_stream                        # 监听 tcp://127.0.0.1:5555
//   ./tts_stream -a tcp://0.0.0.0:5560  # 自定义地址
//   ./tts_stream --speed 1.2            # 语速 1.2x

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <zmq.h>
#include "sherpa-onnx/c-api/c-api.h"

#define MINIAUDIO_IMPLEMENTATION
#include "third_party/miniaudio.h"

// ═══════════════════════════════════════════════════════════════
// 配置
// ═══════════════════════════════════════════════════════════════
struct Config {
    std::string model_dir = "./matcha-icefall-zh-baker";
    std::string vocoder;
    std::string zmq_addr = "tcp://127.0.0.1:5555";
    float speed = 1.0f;
    int num_threads = 2;
    int queue_max = 8;
    bool debug = false;
};

// ═══════════════════════════════════════════════════════════════
// 无锁环形缓冲区 (单生产者单消费者, miniaudio 回调线程安全)
// ═══════════════════════════════════════════════════════════════
class AudioRingBuffer {
    std::vector<float> buf_;
    size_t cap_;
    std::atomic<size_t> read_{0};
    std::atomic<size_t> write_{0};
    std::atomic<uint64_t> total_consumed_{0};

public:
    explicit AudioRingBuffer(size_t capacity)
        : buf_(capacity, 0.0f), cap_(capacity) {}

    // 可写空间 (供生产者使用)
    size_t writable() const {
        size_t r = read_.load(std::memory_order_acquire);
        size_t w = write_.load(std::memory_order_relaxed);
        return (w >= r) ? (cap_ - w + r - 1) : (r - w - 1);
    }

    // 可读数据 (供回调使用)
    size_t readable() const {
        size_t r = read_.load(std::memory_order_relaxed);
        size_t w = write_.load(std::memory_order_acquire);
        return (w >= r) ? (w - r) : (cap_ - r + w);
    }

    // 写入数据 (生产者: 播放线程), 返回实际写入帧数
    size_t write_data(const float *data, size_t count) {
        size_t w = write_.load(std::memory_order_relaxed);
        size_t r = read_.load(std::memory_order_acquire);
        size_t avail = (w >= r) ? (cap_ - w + r - 1) : (r - w - 1);
        count = std::min(count, avail);
        if (count == 0) return 0;

        size_t first = std::min(count, cap_ - w);
        std::memcpy(&buf_[w], data, first * sizeof(float));
        if (count > first)
            std::memcpy(&buf_[0], data + first, (count - first) * sizeof(float));

        write_.store((w + count) % cap_, std::memory_order_release);
        return count;
    }

    // 读取数据 (消费者: miniaudio 回调), 不足部分填静音
    size_t read_data(float *out, size_t count) {
        size_t r = read_.load(std::memory_order_relaxed);
        size_t w = write_.load(std::memory_order_acquire);
        size_t avail = (w >= r) ? (w - r) : (cap_ - r + w);
        size_t to_read = std::min(count, avail);

        if (to_read > 0) {
            size_t first = std::min(to_read, cap_ - r);
            std::memcpy(out, &buf_[r], first * sizeof(float));
            if (to_read > first)
                std::memcpy(out + first, &buf_[0], (to_read - first) * sizeof(float));
            read_.store((r + to_read) % cap_, std::memory_order_release);
            total_consumed_.fetch_add(to_read, std::memory_order_relaxed);
        }
        if (to_read < count)
            std::memset(out + to_read, 0, (count - to_read) * sizeof(float));
        return to_read;
    }

    // 等待直到有足够空间
    void wait_for_space(size_t needed, int timeout_ms = 5) {
        while (writable() < needed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        }
    }

    // 等待直到缓冲区排空
    void drain() {
        while (readable() > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    uint64_t total_consumed() const {
        return total_consumed_.load(std::memory_order_relaxed);
    }
};

// ═══════════════════════════════════════════════════════════════
// 线程安全队列 (带背压)
// ═══════════════════════════════════════════════════════════════
template <typename T>
class BoundedQueue {
    std::queue<T> q_;
    std::mutex mtx_;
    std::condition_variable cv_;
    size_t max_size_;
    bool closed_ = false;

public:
    explicit BoundedQueue(size_t max_size) : max_size_(max_size) {}

    bool push(T item) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [&] { return q_.size() < max_size_ || closed_; });
        if (closed_) return false;
        q_.push(std::move(item));
        cv_.notify_all();
        return true;
    }

    bool pop(T &item) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [&] { return !q_.empty() || closed_; });
        if (q_.empty()) return false;
        item = std::move(q_.front());
        q_.pop();
        cv_.notify_all();
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mtx_);
        closed_ = true;
        cv_.notify_all();
    }
};

// ═══════════════════════════════════════════════════════════════
// 中文分句器
// ═══════════════════════════════════════════════════════════════
static bool is_sentence_end(const std::string &text, size_t pos) {
    if (pos >= text.size()) return false;
    // UTF-8 中文标点: 。！？；
    // ASCII 标点: . ! ? ; \n
    char c = text[pos];
    if (c == '.' || c == '!' || c == '?' || c == ';' || c == '\n')
        return true;
    // UTF-8 三字节中文标点检测
    if (pos + 2 < text.size()) {
        unsigned char b0 = (unsigned char)text[pos];
        unsigned char b1 = (unsigned char)text[pos + 1];
        unsigned char b2 = (unsigned char)text[pos + 2];
        // 。= E3 80 82, ！= EF BC 81, ？= EF BC 9F, ；= EF BC 9B
        if (b0 == 0xE3 && b1 == 0x80 && b2 == 0x82) return true;  // 。
        if (b0 == 0xEF && b1 == 0xBC) {
            if (b2 == 0x81 || b2 == 0x9F || b2 == 0x9B) return true;  // ！？；
        }
    }
    return false;
}

static std::vector<std::string> split_sentences(const std::string &text) {
    std::vector<std::string> result;
    size_t start = 0;
    for (size_t i = 0; i < text.size(); ++i) {
        if (is_sentence_end(text, i)) {
            // 包含标点 (UTF-8 可能是3字节)
            size_t char_len = 1;
            unsigned char b0 = (unsigned char)text[i];
            if (b0 >= 0xE0 && b0 < 0xF0 && i + 2 < text.size()) char_len = 3;
            size_t end = i + char_len;
            std::string sentence = text.substr(start, end - start);
            // 去除首尾空白
            size_t fs = sentence.find_first_not_of(" \t\r\n");
            size_t ls = sentence.find_last_not_of(" \t\r\n");
            if (fs != std::string::npos) {
                sentence = sentence.substr(fs, ls - fs + 1);
                if (!sentence.empty()) result.push_back(sentence);
            }
            start = end;
        }
    }
    // 最后一段（没有句号结尾）
    if (start < text.size()) {
        std::string tail = text.substr(start);
        size_t fs = tail.find_first_not_of(" \t\r\n");
        size_t ls = tail.find_last_not_of(" \t\r\n");
        if (fs != std::string::npos) {
            tail = tail.substr(fs, ls - fs + 1);
            if (!tail.empty()) result.push_back(tail);
        }
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════
// miniaudio 单一设备 + 环形缓冲区
// ═══════════════════════════════════════════════════════════════
static AudioRingBuffer *g_ring_buf = nullptr;

static void ring_buffer_callback(ma_device *device, void *output,
                                 const void * /*input*/,
                                 ma_uint32 frame_count) {
    float *out = static_cast<float *>(output);
    g_ring_buf->read_data(out, frame_count);
}

// ═══════════════════════════════════════════════════════════════
// 合成线程 (生产者)
// ═══════════════════════════════════════════════════════════════
static void synthesize_worker(
    const SherpaOnnxOfflineTts *tts,
    float speed,
    float silence_scale,
    BoundedQueue<std::string> &sentence_queue,
    AudioRingBuffer &audio_ring) {

    std::string sentence;
    int sent_idx = 0;

    while (sentence_queue.pop(sentence)) {
        sent_idx++;
        fprintf(stderr, "[合成#%d] \"%s\"\n", sent_idx, sentence.c_str());

        SherpaOnnxGenerationConfig gc;
        memset(&gc, 0, sizeof(gc));
        gc.speed = speed;
        gc.silence_scale = silence_scale;

        const SherpaOnnxGeneratedAudio *audio =
            SherpaOnnxOfflineTtsGenerateWithConfig(tts, sentence.c_str(),
                                                   &gc, nullptr, nullptr);
        if (audio && audio->n > 0) {
            float dur = (float)audio->n / audio->sample_rate;
            fprintf(stderr, "[合成#%d] 完成 %.2fs\n", sent_idx, dur);

            // 阻塞写入环形缓冲区 (合成速度 > 消费速度时会短暂等待)
            size_t total_written = 0;
            while (total_written < (size_t)audio->n) {
                audio_ring.wait_for_space(1024);
                size_t wrote = audio_ring.write_data(
                    audio->samples + total_written,
                    (size_t)audio->n - total_written);
                total_written += wrote;
            }
        } else {
            fprintf(stderr, "[合成#%d] 失败\n", sent_idx);
        }

        if (audio) SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
    }

    fprintf(stderr, "[合成] 全部完成，共 %d 句\n", sent_idx);
}

// 播放线程已合并到合成线程 (合成后直接写入环形缓冲区)
// miniaudio 设备回调从环形缓冲区读取数据 → 扬声器

// ═══════════════════════════════════════════════════════════════
// 命令行解析
// ═══════════════════════════════════════════════════════════════
static void usage(const char *prog) {
    fprintf(stderr,
        "流水线 TTS — ZMQ 接收文本，边合成边播放\n\n"
        "用法: %s [选项]\n\n"
        "选项:\n"
        "  -a, --addr ADDR       ZMQ 订阅地址 (默认: tcp://127.0.0.1:5555)\n"
        "  -s, --speed FLOAT     语速 (默认: 1.0)\n"
        "  -m, --model-dir DIR   模型目录 (默认: ./matcha-icefall-zh-baker)\n"
        "  -t, --num-threads N   ONNX 线程数 (默认: 2)\n"
        "  -q, --queue-max N     音频队列上限 (默认: 8)\n"
        "  -d, --debug           调试模式\n"
        "  -h, --help            帮助\n\n"
        "示例:\n"
        "  # 终端1: 启动 TTS\n"
        "  %s\n\n"
        "  # 终端2: 发送文本\n"
        "  python3 publish_text.py\n",
        prog, prog);
}

static int parse_args(int argc, char *argv[], Config &cfg) {
    static struct option opts[] = {
        {"addr",        required_argument, 0, 'a'},
        {"speed",       required_argument, 0, 's'},
        {"model-dir",   required_argument, 0, 'm'},
        {"num-threads", required_argument, 0, 't'},
        {"queue-max",   required_argument, 0, 'q'},
        {"debug",       no_argument,       0, 'd'},
        {"help",        no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    int c;
    while ((c = getopt_long(argc, argv, "a:s:m:t:q:dh", opts, nullptr)) != -1) {
        switch (c) {
            case 'a': cfg.zmq_addr = optarg; break;
            case 's': cfg.speed = atof(optarg); break;
            case 'm': cfg.model_dir = optarg; break;
            case 't': cfg.num_threads = atoi(optarg); break;
            case 'q': cfg.queue_max = atoi(optarg); break;
            case 'd': cfg.debug = true; break;
            case 'h': usage(argv[0]); exit(0);
            default:  usage(argv[0]); return -1;
        }
    }
    if (cfg.vocoder.empty())
        cfg.vocoder = cfg.model_dir + "/vocos-22khz-univ.onnx";
    return 0;
}

// ═══════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════
int main(int argc, char *argv[]) {
    Config cfg;
    if (parse_args(argc, argv, cfg) != 0) return 1;

    // ── 1. 初始化 TTS 引擎 ──
    auto acoustic = cfg.model_dir + "/model-steps-3.onnx";
    auto lexicon  = cfg.model_dir + "/lexicon.txt";
    auto tokens   = cfg.model_dir + "/tokens.txt";
    auto dict_dir = cfg.model_dir + "/dict";
    auto rules    = cfg.model_dir + "/phone.fst," +
                    cfg.model_dir + "/date.fst," +
                    cfg.model_dir + "/number.fst";

    SherpaOnnxOfflineTtsConfig tts_cfg;
    memset(&tts_cfg, 0, sizeof(tts_cfg));
    tts_cfg.model.matcha.acoustic_model = acoustic.c_str();
    tts_cfg.model.matcha.vocoder        = cfg.vocoder.c_str();
    tts_cfg.model.matcha.lexicon        = lexicon.c_str();
    tts_cfg.model.matcha.tokens         = tokens.c_str();
    tts_cfg.model.matcha.dict_dir       = dict_dir.c_str();
    tts_cfg.model.num_threads           = cfg.num_threads;
    tts_cfg.model.debug                 = cfg.debug ? 1 : 0;
    tts_cfg.rule_fsts                   = rules.c_str();
    tts_cfg.max_num_sentences           = 1;  // 逐句处理

    const SherpaOnnxOfflineTts *tts = SherpaOnnxCreateOfflineTts(&tts_cfg);
    if (!tts) {
        fprintf(stderr, "错误: TTS 引擎创建失败\n");
        return 1;
    }
    fprintf(stderr, "[TTS] 引擎就绪 (采样率: %dHz)\n",
            SherpaOnnxOfflineTtsSampleRate(tts));

    // ── 2. 初始化 ZMQ ──
    void *zmq_ctx = zmq_ctx_new();
    void *sub_sock = zmq_socket(zmq_ctx, ZMQ_SUB);
    zmq_setsockopt(sub_sock, ZMQ_SUBSCRIBE, "", 0);  // 订阅所有消息

    if (zmq_connect(sub_sock, cfg.zmq_addr.c_str()) != 0) {
        fprintf(stderr, "错误: ZMQ 连接 %s 失败: %s\n",
                cfg.zmq_addr.c_str(), zmq_strerror(zmq_errno()));
        SherpaOnnxDestroyOfflineTts(tts);
        zmq_close(sub_sock);
        zmq_ctx_destroy(zmq_ctx);
        return 1;
    }
    fprintf(stderr, "[ZMQ] 已连接 %s (等待文本...)\n", cfg.zmq_addr.c_str());
    fprintf(stderr, "───────────────────────────────────────────\n");

    // ── 3. 初始化音频设备 (单一设备, 全程运行) ──
    // 环形缓冲区: 容纳约 10 秒音频 (22050Hz × 10s)
    const size_t ring_cap = static_cast<size_t>(
        SherpaOnnxOfflineTtsSampleRate(tts)) * 10;
    AudioRingBuffer audio_ring(ring_cap);
    g_ring_buf = &audio_ring;

    int32_t sample_rate = SherpaOnnxOfflineTtsSampleRate(tts);
    ma_device_config dev_cfg = ma_device_config_init(ma_device_type_playback);
    dev_cfg.playback.format = ma_format_f32;
    dev_cfg.playback.channels = 1;
    dev_cfg.sampleRate = static_cast<ma_uint32>(sample_rate);
    dev_cfg.dataCallback = ring_buffer_callback;
    dev_cfg.periodSizeInFrames = 1024;  // ~46ms @ 22050Hz

    ma_device audio_device;
    if (ma_device_init(nullptr, &dev_cfg, &audio_device) != MA_SUCCESS) {
        fprintf(stderr, "错误: 音频设备初始化失败\n");
        SherpaOnnxDestroyOfflineTts(tts);
        zmq_close(sub_sock);
        zmq_ctx_destroy(zmq_ctx);
        return 1;
    }
    ma_device_start(&audio_device);
    fprintf(stderr, "[音频] 设备就绪 (%dHz, 缓冲区 %zu 帧 ≈ %.1fs)\n",
            sample_rate, ring_cap, (float)ring_cap / sample_rate);

    // ── 4. 创建队列 ──
    BoundedQueue<std::string> sentence_queue(
        static_cast<size_t>(cfg.queue_max * 2));

    // ── 5. 启动合成线程 (合成后直接写入环形缓冲区) ──
    std::thread synthesizer(synthesize_worker, tts, cfg.speed, 0.2f,
                            std::ref(sentence_queue), std::ref(audio_ring));

    // ── 6. 主循环: 接收 ZMQ 文本 → 分句 → 入队 ──
    int msg_count = 0;
    int total_sentences = 0;
    bool running = true;

    while (running) {
        // 接收 ZMQ 消息 (阻塞)
        char buf[65536];
        int nbytes = zmq_recv(sub_sock, buf, sizeof(buf) - 1, 0);
        if (nbytes < 0) {
            if (zmq_errno() == EINTR) continue;
            fprintf(stderr, "[ZMQ] 接收错误: %s\n", zmq_strerror(zmq_errno()));
            break;
        }
        buf[nbytes] = '\0';
        msg_count++;

        std::string text(buf, static_cast<size_t>(nbytes));

        // 检查退出命令
        if (text == "__EXIT__" || text == "__exit__") {
            fprintf(stderr, "[主循环] 收到退出指令\n");
            running = false;
            break;
        }

        // 分句
        auto sentences = split_sentences(text);
        if (sentences.empty()) {
            fprintf(stderr, "[主循环#%d] 空消息，跳过\n", msg_count);
            continue;
        }

        fprintf(stderr, "[主循环#%d] 收到 %zu 字节 → %zu 句\n",
                msg_count, text.size(), sentences.size());

        for (auto &s : sentences) {
            sentence_queue.push(std::move(s));
            total_sentences++;
        }
    }

    // ── 7. 优雅退出 ──
    fprintf(stderr, "───────────────────────────────────────────\n");
    fprintf(stderr, "[退出] 收到 %d 条消息, %d 句, 等待完成...\n",
            msg_count, total_sentences);

    sentence_queue.close();  // 通知合成线程没有更多句子
    synthesizer.join();      // 等合成完成

    fprintf(stderr, "[排空] 等待音频播放完毕...\n");
    audio_ring.drain();      // 等待环形缓冲区排空

    ma_device_uninit(&audio_device);
    uint64_t total_samples = audio_ring.total_consumed();
    fprintf(stderr, "[完成] 共播放 %.2fs 音频\n",
            (float)total_samples / sample_rate);

    // 清理
    SherpaOnnxDestroyOfflineTts(tts);
    zmq_close(sub_sock);
    zmq_ctx_destroy(zmq_ctx);
    return 0;
}
