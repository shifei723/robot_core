/**
 * Sherpa-ONNX KWS + VAD Demo (C++ 版本)
 *
 * 流程：
 * 1. 持续监听麦克风，用 KWS 检测唤醒词
 * 2. 检测到唤醒词后，切换为 VAD 模式，捕获后续语音命令
 * 3. 静音足够久后，保存命令为 WAV，状态归零
 * 4. 在 commands/kws_status 写入状态（0=未唤醒, 1=已唤醒）
 *
 * 用法：
 *   ./kws_vad_demo --keywords-file keywords_xiaofei.txt --model-dir ../model
 *   ./kws_vad_demo --wav test.wav --keywords-file test_keywords.txt --model-dir ../model
 */

#include "audio_capture.h"
#include "wav_writer.h"

#include <sherpa-onnx/c-api/c-api.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <climits>
#include <unistd.h>
#include <libgen.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

// ---------------------------------------------------------------------------
// 全局状态
// ---------------------------------------------------------------------------
static volatile bool g_running = true;

static void SignalHandler(int) {
    g_running = false;
}

// ---------------------------------------------------------------------------
// 命令行参数
// ---------------------------------------------------------------------------
struct Args {
    std::string tokens = "model/tokens.txt";
    std::string encoder = "model/encoder-epoch-12-avg-2-chunk-16-left-64.onnx";
    std::string decoder = "model/decoder-epoch-12-avg-2-chunk-16-left-64.onnx";
    std::string joiner = "model/joiner-epoch-12-avg-2-chunk-16-left-64.onnx";
    std::string keywords_file = "keywords_xiaofei.txt";
    std::string vad_model = "vad_model/silero_vad.onnx";
    std::string output_dir = "commands";
    std::string status_file = "kws_status";
    std::string wav_file = "";
    std::string model_dir = "model";

    float keywords_score = 3.0f;
    float keywords_threshold = 0.25f;
    float vad_threshold = 0.5f;
    float vad_min_silence = 0.5f;
    float vad_min_speech = 0.25f;
    float max_command_duration = 30.0f;
    // 唤醒应答静默窗口(毫秒)：唤醒后这段时间内不喂 VAD。
    // 默认 0 = 关闭，因为本设备（HK MIC 阵列麦）自带硬件 AEC，实测
    // 播放 440Hz 测试音时录音里该频分量仅 0.12，比环境底噪(6.69)还低约55倍，
    // 机器人听不到自己，无需屏蔽。
    // 若换成无 AEC 的麦克风，需设为 1500 左右，否则“我在”会被当成命令录进去。
    float ack_blank_ms = 0.0f;

    int num_threads = 2;
    int sample_rate = 16000;
    int kws_chunk_size = 1600;  // 100ms @ 16kHz
    int vad_window_size = 512;
    int num_trailing_blanks = 1;

    bool parse(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 < argc) return argv[++i];
                std::cerr << "Missing value for " << arg << std::endl;
                exit(1);
            };
            if (arg == "--tokens") tokens = next();
            else if (arg == "--encoder") encoder = next();
            else if (arg == "--decoder") decoder = next();
            else if (arg == "--joiner") joiner = next();
            else if (arg == "--keywords-file") keywords_file = next();
            else if (arg == "--vad-model") vad_model = next();
            else if (arg == "--output-dir") output_dir = next();
            else if (arg == "--status-file") status_file = next();
            else if (arg == "--wav") wav_file = next();
            else if (arg == "--model-dir") model_dir = next();
            else if (arg == "--keywords-score") keywords_score = std::stof(next());
            else if (arg == "--keywords-threshold") keywords_threshold = std::stof(next());
            else if (arg == "--vad-threshold") vad_threshold = std::stof(next());
            else if (arg == "--vad-min-silence-duration") vad_min_silence = std::stof(next());
            else if (arg == "--vad-min-speech-duration") vad_min_speech = std::stof(next());
            else if (arg == "--max-command-duration") max_command_duration = std::stof(next());
            else if (arg == "--ack-blank-ms") ack_blank_ms = std::stof(next());
            else if (arg == "--num-threads") num_threads = std::stoi(next());
            else if (arg == "--sample-rate") sample_rate = std::stoi(next());
            else if (arg == "--kws-chunk-size") kws_chunk_size = std::stoi(next());
            else if (arg == "--vad-window-size") vad_window_size = std::stoi(next());
            else if (arg == "--num-trailing-blanks") num_trailing_blanks = std::stoi(next());
            else if (arg == "-h" || arg == "--help") {
                std::cout << "Usage: kws_vad_demo [options]\n"
                          << "  --keywords-file FILE   Wake word file\n"
                          << "  --model-dir DIR        Model directory\n"
                          << "  --wav FILE             Offline test WAV file\n"
                          << "  --output-dir DIR       Output directory (default: commands)\n"
                          << "  See source for all options.\n";
                exit(0);
            } else {
                std::cerr << "Unknown argument: " << arg << std::endl;
                return false;
            }
        }

        // 如果指定了 model-dir，自动拼接路径
        if (!model_dir.empty()) {
            auto resolve = [&](std::string& path) {
                if (path.find('/') == std::string::npos && path.find('\\') == std::string::npos) {
                    path = model_dir + "/" + path;
                }
            };
            resolve(tokens);
            resolve(encoder);
            resolve(decoder);
            resolve(joiner);
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// 状态文件写入
// ---------------------------------------------------------------------------
static void WriteStatus(const std::string& path, int value) {
    std::string tmp = path + ".tmp";
    std::ofstream f(tmp);
    if (f.is_open()) {
        f << value;
        f.close();
        std::rename(tmp.c_str(), path.c_str());
    }
}

// ---------------------------------------------------------------------------
// WAV 文件读取（离线测试用）
// ---------------------------------------------------------------------------
struct WavData {
    std::vector<float> samples;
    int sample_rate = 0;
};

static WavData ReadWav(const std::string& path) {
    WavData result;
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return result;

    char buf[4];
    f.read(buf, 4);
    if (std::memcmp(buf, "RIFF", 4) != 0) return result;
    f.read(buf, 4);  // file size
    f.read(buf, 4);
    if (std::memcmp(buf, "WAVE", 4) != 0) return result;

    uint16_t channels = 0, bits = 0;
    uint32_t rate = 0, data_size = 0;

    while (f.read(buf, 4)) {
        uint32_t chunk_size = 0;
        f.read(reinterpret_cast<char*>(&chunk_size), 4);

        if (std::memcmp(buf, "fmt ", 4) == 0) {
            f.read(buf, 2);  // format
            f.read(reinterpret_cast<char*>(&channels), 2);
            f.read(reinterpret_cast<char*>(&rate), 4);
            f.read(buf, 4);  // byte rate
            f.read(buf, 2);  // block align
            f.read(reinterpret_cast<char*>(&bits), 2);
            if (chunk_size > 16) f.seekg(chunk_size - 16, std::ios::cur);
        } else if (std::memcmp(buf, "data", 4) == 0) {
            data_size = chunk_size;
            break;
        } else {
            f.seekg(chunk_size, std::ios::cur);
        }
    }

    if (channels != 1 || bits != 16) return result;
    result.sample_rate = static_cast<int>(rate);

    int num_samples = data_size / 2;
    result.samples.resize(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        int16_t v = 0;
        f.read(reinterpret_cast<char*>(&v), 2);
        result.samples[i] = static_cast<float>(v) / 32768.0f;
    }
    return result;
}

// ---------------------------------------------------------------------------
// 主程序
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    Args args;
    if (!args.parse(argc, argv)) return 1;

    // 创建输出目录
    std::string mkdir_cmd = "mkdir -p " + args.output_dir;
    system(mkdir_cmd.c_str());

    std::string status_path = args.output_dir + "/" + args.status_file;
    std::string output_path = args.output_dir + "/latest_command.wav";

    WriteStatus(status_path, 0);

    // ----- 初始化 KWS（Keyword Spotter，与 Python sherpa_onnx.KeywordSpotter 一致）-----
    std::cout << "[INIT] Loading KWS model..." << std::endl;

    SherpaOnnxOnlineTransducerModelConfig transducer_config;
    std::memset(&transducer_config, 0, sizeof(transducer_config));
    transducer_config.encoder = args.encoder.c_str();
    transducer_config.decoder = args.decoder.c_str();
    transducer_config.joiner = args.joiner.c_str();

    SherpaOnnxOnlineModelConfig model_config;
    std::memset(&model_config, 0, sizeof(model_config));
    model_config.transducer = transducer_config;
    model_config.tokens = args.tokens.c_str();
    model_config.num_threads = args.num_threads;
    model_config.provider = "cpu";
    model_config.debug = 0;

    SherpaOnnxKeywordSpotterConfig kws_config;
    std::memset(&kws_config, 0, sizeof(kws_config));
    kws_config.feat_config.sample_rate = args.sample_rate;
    kws_config.feat_config.feature_dim = 80;
    kws_config.model_config = model_config;
    kws_config.max_active_paths = 4;
    kws_config.num_trailing_blanks = args.num_trailing_blanks;
    kws_config.keywords_score = args.keywords_score;
    kws_config.keywords_threshold = args.keywords_threshold;
    kws_config.keywords_file = args.keywords_file.c_str();

    auto* spotter = SherpaOnnxCreateKeywordSpotter(&kws_config);
    if (!spotter) {
        std::cerr << "[ERROR] Failed to create Keyword Spotter" << std::endl;
        return 1;
    }
    auto* kws_stream = SherpaOnnxCreateKeywordStream(spotter);
    if (!kws_stream) {
        std::cerr << "[ERROR] Failed to create KWS stream" << std::endl;
        SherpaOnnxDestroyKeywordSpotter(spotter);
        return 1;
    }

    // ----- 初始化 VAD -----
    std::cout << "[INIT] Loading VAD model..." << std::endl;

    SherpaOnnxVadModelConfig vad_config;
    std::memset(&vad_config, 0, sizeof(vad_config));
    vad_config.silero_vad.model = args.vad_model.c_str();
    vad_config.silero_vad.threshold = args.vad_threshold;
    vad_config.silero_vad.min_silence_duration = args.vad_min_silence;
    vad_config.silero_vad.min_speech_duration = args.vad_min_speech;
    vad_config.silero_vad.window_size = args.vad_window_size;
    vad_config.sample_rate = args.sample_rate;
    vad_config.num_threads = args.num_threads;
    vad_config.provider = "cpu";

    auto* vad = SherpaOnnxCreateVoiceActivityDetector(&vad_config, 60.0f);
    if (!vad) {
        std::cerr << "[ERROR] Failed to create VAD detector" << std::endl;
        SherpaOnnxDestroyOnlineStream(kws_stream);
        SherpaOnnxDestroyKeywordSpotter(spotter);
        return 1;
    }

    // ----- 音频输入 -----
    WavData wav_data;
    AudioCapture* mic = nullptr;

    if (!args.wav_file.empty()) {
        wav_data = ReadWav(args.wav_file);
        if (wav_data.samples.empty()) {
            std::cerr << "[ERROR] Failed to read WAV file: " << args.wav_file << std::endl;
            SherpaOnnxDestroyVoiceActivityDetector(vad);
            SherpaOnnxDestroyOnlineStream(kws_stream);
            SherpaOnnxDestroyKeywordSpotter(spotter);
            return 1;
        }
        std::cout << "[INIT] Using WAV file: " << args.wav_file
                  << " (" << wav_data.samples.size() / args.sample_rate << "s)" << std::endl;
    } else {
        try {
            mic = new AudioCapture(args.sample_rate, args.kws_chunk_size, 1);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Audio capture failed: " << e.what() << std::endl;
            SherpaOnnxDestroyVoiceActivityDetector(vad);
            SherpaOnnxDestroyOnlineStream(kws_stream);
            SherpaOnnxDestroyKeywordSpotter(spotter);
            return 1;
        }
        std::cout << "[INIT] Microphone opened" << std::endl;
    }

    // ----- 状态机 -----
    enum State { IDLE, LISTENING };
    State state = IDLE;
    auto listening_start = std::chrono::steady_clock::now();
    int command_count = 0;

    std::cout << "[INFO] Keywords file: " << args.keywords_file << std::endl;
    std::cout << "[INFO] Status file:   " << status_path << std::endl;
    std::cout << "[INFO] Output file:   " << output_path << std::endl;
    std::cout << "[INFO] Ack blank:     " << args.ack_blank_ms
              << " ms (0=不屏蔽，本设备有硬件 AEC)" << std::endl;
    if (mic) {
        std::cout << "[INFO] Say wake word, then speak command. Ctrl+C to stop.\n" << std::endl;
    } else {
        std::cout << "[INFO] Processing file chunks...\n" << std::endl;
    }

    auto finalize_command = [&]() {
        SherpaOnnxVoiceActivityDetectorFlush(vad);
        if (!SherpaOnnxVoiceActivityDetectorEmpty(vad)) {
            auto* seg = SherpaOnnxVoiceActivityDetectorFront(vad);
            if (seg && seg->n > 0) {
                std::vector<float> samples(seg->samples, seg->samples + seg->n);
                float dur = static_cast<float>(samples.size()) / args.sample_rate;
                SaveWav(output_path, samples, args.sample_rate);
                std::cout << "[VAD] 命令已保存: " << output_path
                          << " (" << dur << "s) [覆盖更新]" << std::endl;
                SherpaOnnxDestroySpeechSegment(seg);
            } else {
                std::cout << "[VAD] 未检测到有效语音" << std::endl;
            }
        } else {
            std::cout << "[VAD] 未检测到有效语音" << std::endl;
        }
        SherpaOnnxVoiceActivityDetectorReset(vad);
        WriteStatus(status_path, 0);
        std::cout << "[STATUS] IDLE (命令结束) -> " << status_path << std::endl;

        SherpaOnnxDestroyOnlineStream(kws_stream);
        kws_stream = SherpaOnnxCreateKeywordStream(spotter);
    };

    // 离线 WAV 处理偏移
    size_t wav_offset = 0;

    while (g_running) {
        std::vector<float> chunk;

        if (mic) {
            try {
                chunk = mic->Read();
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Audio read failed: " << e.what() << std::endl;
                break;
            }
        } else {
            size_t remaining = wav_data.samples.size() - wav_offset;
            size_t n = std::min(static_cast<size_t>(args.kws_chunk_size), remaining);
            if (n == 0) {
                if (state == LISTENING) {
                    std::cout << "[VAD] 文件结束，尝试收尾" << std::endl;
                    finalize_command();
                    state = IDLE;
                }
                break;
            }
            chunk.assign(wav_data.samples.begin() + wav_offset,
                         wav_data.samples.begin() + wav_offset + n);
            wav_offset += n;
        }

        if (state == IDLE) {
            // KWS 检测
            SherpaOnnxOnlineStreamAcceptWaveform(kws_stream, args.sample_rate,
                                                  chunk.data(), chunk.size());
            while (SherpaOnnxIsKeywordStreamReady(spotter, kws_stream)) {
                SherpaOnnxDecodeKeywordStream(spotter, kws_stream);
            }

            auto* result = SherpaOnnxGetKeywordResult(spotter, kws_stream);
            if (result && result->keyword && std::strlen(result->keyword) > 0) {
                std::string text = result->keyword;
                SherpaOnnxDestroyKeywordResult(result);

                std::cout << "[KWS] 唤醒词检测到: " << text << std::endl;
                WriteStatus(status_path, 1);
                std::cout << "[STATUS] LISTENING (已唤醒) -> " << status_path << std::endl;

                SherpaOnnxResetKeywordStream(spotter, kws_stream);
                SherpaOnnxVoiceActivityDetectorReset(vad);
                state = LISTENING;
                listening_start = std::chrono::steady_clock::now();
            } else {
                if (result) SherpaOnnxDestroyKeywordResult(result);
            }
        } else {
            // 唤醒应答静默窗口（默认 0 = 不启用，本设备有硬件 AEC）
            if (args.ack_blank_ms > 0.0f) {
                auto now = std::chrono::steady_clock::now();
                float since_wake_ms =
                    std::chrono::duration<float, std::milli>(now - listening_start).count();
                if (since_wake_ms < args.ack_blank_ms) {
                    continue;
                }
            }

            // VAD 检测
            SherpaOnnxVoiceActivityDetectorAcceptWaveform(vad, chunk.data(), chunk.size());

            if (!SherpaOnnxVoiceActivityDetectorEmpty(vad)) {
                finalize_command();
                state = IDLE;
                continue;
            }

            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(now - listening_start).count();
            if (elapsed >= args.max_command_duration) {
                std::cout << "[VAD] 等待命令超时" << std::endl;
                finalize_command();
                state = IDLE;
            }
        }
    }

    // ----- 清理 -----
    std::cout << "\n[INFO] Shutting down..." << std::endl;
    WriteStatus(status_path, 0);

    if (mic) delete mic;
    if (kws_stream) SherpaOnnxDestroyOnlineStream(kws_stream);
    if (spotter) SherpaOnnxDestroyKeywordSpotter(spotter);
    if (vad) SherpaOnnxDestroyVoiceActivityDetector(vad);

    std::cout << "[INFO] Done." << std::endl;
    return 0;
}
