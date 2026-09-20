// tts.cpp - Matcha TTS 中文语音合成 (sherpa-onnx C API)
//
// 用法:
//   ./tts "你好，欢迎使用语音合成"
//   ./tts -o hello.wav -s 1.2 "100人参加活动"
//   ./tts --model-dir=./models --num-threads=4 "你的文本"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <getopt.h>
#include "sherpa-onnx/c-api/c-api.h"

struct Config {
    std::string model_dir = "./matcha-icefall-zh-baker";
    std::string vocoder;
    std::string output = "tts_output.wav";
    std::string text;
    float speed = 1.0f;
    int num_threads = 2;
    bool debug = false;
};

void usage(const char *prog) {
    fprintf(stderr,
        "Matcha TTS 中文语音合成 (sherpa-onnx C API)\n\n"
        "用法: %s [选项] \"要合成的文本\"\n\n"
        "选项:\n"
        "  -o, --output FILE     输出 WAV (默认: tts_output.wav)\n"
        "  -s, --speed FLOAT     语速 >1快 <1慢 (默认: 1.0)\n"
        "  -m, --model-dir DIR   模型目录 (默认: ./matcha-icefall-zh-baker)\n"
        "  -v, --vocoder FILE    声码器路径 (默认: model-dir/vocos-22khz-univ.onnx)\n"
        "  -t, --num-threads N   线程数 (默认: 2)\n"
        "  -d, --debug           显示调试信息\n"
        "  -h, --help            显示帮助\n\n"
        "示例:\n"
        "  %s \"你好世界\"\n"
        "  %s -o hello.wav -s 1.2 \"100人参加活动\"\n",
        prog, prog, prog);
}

int parse_args(int argc, char *argv[], Config &cfg) {
    static struct option opts[] = {
        {"output",      required_argument, 0, 'o'},
        {"speed",       required_argument, 0, 's'},
        {"model-dir",   required_argument, 0, 'm'},
        {"vocoder",     required_argument, 0, 'v'},
        {"num-threads", required_argument, 0, 't'},
        {"debug",       no_argument,       0, 'd'},
        {"help",        no_argument,       0, 'h'},
        {0,0,0,0}
    };
    int c;
    while ((c = getopt_long(argc, argv, "o:s:m:v:t:dh", opts, NULL)) != -1) {
        switch (c) {
            case 'o': cfg.output = optarg; break;
            case 's': cfg.speed = atof(optarg); break;
            case 'm': cfg.model_dir = optarg; break;
            case 'v': cfg.vocoder = optarg; break;
            case 't': cfg.num_threads = atoi(optarg); break;
            case 'd': cfg.debug = true; break;
            case 'h': usage(argv[0]); exit(0);
            default:  usage(argv[0]); return -1;
        }
    }
    if (optind < argc) cfg.text = argv[optind];
    if (cfg.text.empty()) {
        fprintf(stderr, "错误: 未提供文本\n");
        usage(argv[0]);
        return -1;
    }
    if (cfg.vocoder.empty())
        cfg.vocoder = cfg.model_dir + "/vocos-22khz-univ.onnx";
    return 0;
}

static int progress_cb(const float*, int32_t, float p, void*) {
    fprintf(stderr, "\r进度: %.1f%%", p * 100);
    fflush(stderr);
    return 1;
}

int main(int argc, char *argv[]) {
    Config cfg;
    if (parse_args(argc, argv, cfg) != 0) return 1;

    auto acoustic = cfg.model_dir + "/model-steps-3.onnx";
    auto lexicon  = cfg.model_dir + "/lexicon.txt";
    auto tokens   = cfg.model_dir + "/tokens.txt";
    auto dict_dir = cfg.model_dir + "/dict";
    auto rules    = cfg.model_dir + "/phone.fst," +
                    cfg.model_dir + "/date.fst," +
                    cfg.model_dir + "/number.fst";

    SherpaOnnxOfflineTtsConfig c;
    memset(&c, 0, sizeof(c));
    c.model.matcha.acoustic_model = acoustic.c_str();
    c.model.matcha.vocoder        = cfg.vocoder.c_str();
    c.model.matcha.lexicon        = lexicon.c_str();
    c.model.matcha.tokens         = tokens.c_str();
    c.model.matcha.dict_dir       = dict_dir.c_str();
    c.model.num_threads           = cfg.num_threads;
    c.model.debug                 = cfg.debug ? 1 : 0;
    c.rule_fsts                   = rules.c_str();

    auto *tts = SherpaOnnxCreateOfflineTts(&c);
    if (!tts) { fprintf(stderr, "错误: TTS 创建失败\n"); return 1; }

    SherpaOnnxGenerationConfig gc = {0};
    gc.sid = 0;
    gc.speed = cfg.speed;
    gc.silence_scale = 0.2f;

    fprintf(stderr, "文本: %s\n", cfg.text.c_str());
    auto *audio = SherpaOnnxOfflineTtsGenerateWithConfig(
        tts, cfg.text.c_str(), &gc, progress_cb, NULL);
    fprintf(stderr, "\n");

    if (!audio || audio->n == 0) {
        fprintf(stderr, "错误: 生成失败\n");
        SherpaOnnxDestroyOfflineTts(tts);
        return 1;
    }

    SherpaOnnxWriteWave(audio->samples, audio->n, audio->sample_rate,
                        cfg.output.c_str());
    fprintf(stderr, "已保存: %s (%.2fs, %dHz)\n",
            cfg.output.c_str(),
            (float)audio->n / audio->sample_rate,
            audio->sample_rate);

    SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
    SherpaOnnxDestroyOfflineTts(tts);
    return 0;
}
