// Copyright (c) [2025] [Horizon Robotics][Horizon Bole].
//
// You can use this software according to the terms and conditions of
// the Apache v2.0.
// You may obtain a copy of Apache v2.0. at:
//
//     http: //www.apache.org/licenses/LICENSE-2.0
//
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See Apache v2.0 for more details.

#include <stdio.h>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

#include <omp.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>  // NOLINT
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "xlm.h"  // NOLINT

const float FRAME_FACTOR = 2.0f;
const int32_t FPS_MAX_FRAMES = 10;
const int32_t FRAME_MIN = 2;
const int32_t SAMPLE_RATE = 16000;

const std::unordered_map<int32_t, xlm_infer_backend> kModelTypeMap = {
    {0, XLM_INFER_BACKEND_BPU_0},
    {1, XLM_INFER_BACKEND_BPU_1},
    {2, XLM_INFER_BACKEND_BPU_2},
    {3, XLM_INFER_BACKEND_BPU_3},
};

void callback(xlm_result_s* result, xlm_state_e state, void* userdata) {
  if (state == XLM_STATE_END) {
    std::cout << "\n[User] <<< " << std::flush;
  } else if (state == XLM_STATE_ERROR) {
    std::cout << "run error" << std::endl;
  } else if (state == XLM_STATE_START) {
    std::cout << "[Assistant] >>> " << result->text << std::flush;
  } else {
    std::cout << result->text << std::flush;
  }
}

void print_usage(int32_t argc, char **argv) {
  std::cout << "useage:\n./oellm_omni_online --config_path config_path"
            << std::endl;
}

void online_demo(xlm_handle_t omni_handle, int32_t bpu_core, int32_t demo_num,
                 std::string video_path, std::string user_text) {
  xlm_input_s input;
  memset(&input, 0, sizeof(xlm_input_s));
  input.request_num = 1;
  std::vector<xlm_lm_request_t> requests(input.request_num);
  input.requests = requests.data();
  auto& request = input.requests[0];
  memset(&request, 0, sizeof(xlm_lm_request_t));
  request.type = XLM_INPUT_PROMPT;
  request.system_prompt = nullptr;
  if (bpu_core == -1) {
    request.infer_backend = XLM_INFER_BACKEND_BPU_ANY;
  } else {
    request.infer_backend = kModelTypeMap.at(bpu_core);
  }

  // 视频文件路径
  const char* input_filename = video_path.c_str();
  AVFormatContext* fmt_ctx = nullptr;
  if (avformat_open_input(&fmt_ctx, input_filename, nullptr, nullptr) < 0) {
    std::cerr << "Could not open input file.\n";
  }

  if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
    std::cerr << "Could not find stream info.\n";
  }

  int32_t video_stream_index = -1, audio_stream_index = -1;
  for (unsigned int i = 0; i < fmt_ctx->nb_streams; ++i) {
    if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
        video_stream_index < 0)
      video_stream_index = i;
    else if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
             audio_stream_index < 0)
      audio_stream_index = i;
  }

  if (video_stream_index == -1) {
    std::cerr << "No video stream found.\n";
  }

  // Get total frames
  int64_t total_frames = 0;
  int32_t nframes = 0;
  float video_fps = 0.0f;
  if (video_stream_index != -1) {
    AVStream* stream = fmt_ctx->streams[video_stream_index];
    total_frames = stream->nb_frames;
    video_fps = av_q2d(stream->avg_frame_rate);
    // 如果 nb_frames 为 0，可以用时长和帧率估算：
    if (total_frames == 0 && stream->avg_frame_rate.num &&
        stream->avg_frame_rate.den) {
      double duration_sec = 0.0;
      if (fmt_ctx->duration != AV_NOPTS_VALUE) {
        duration_sec = fmt_ctx->duration / (double)AV_TIME_BASE;
      } else {
        std::cout << "Warning try to process first 5s..." << std::endl;
        total_frames = static_cast<int64_t>(video_fps * 5);
      }
      double fps = av_q2d(stream->avg_frame_rate);
      total_frames = static_cast<int64_t>((duration_sec * fps + 0.5));
    }
  }
  nframes = total_frames / video_fps * FRAME_FACTOR;
  nframes = nframes / FRAME_FACTOR * FRAME_FACTOR;
  nframes = std::max(nframes, FRAME_MIN);
  nframes = std::min(nframes, FPS_MAX_FRAMES);

  AVCodecParameters* video_codecpar =
      fmt_ctx->streams[video_stream_index]->codecpar;
  AVCodec* video_codec = avcodec_find_decoder(video_codecpar->codec_id);
  AVCodecContext* video_codec_ctx = avcodec_alloc_context3(video_codec);
  video_codec_ctx->thread_type = FF_THREAD_SLICE;
  video_codec_ctx->thread_count = 4;
  avcodec_parameters_to_context(video_codec_ctx, video_codecpar);
  avcodec_open2(video_codec_ctx, video_codec, nullptr);

  SwsContext* sws_ctx = sws_getContext(
      video_codec_ctx->width, video_codec_ctx->height, video_codec_ctx->pix_fmt,
      video_codec_ctx->width, video_codec_ctx->height, AV_PIX_FMT_NV12,
      SWS_BICUBIC, nullptr, nullptr, nullptr);

  AVFrame* frame = av_frame_alloc();
  AVFrame* frame_nv12 = av_frame_alloc();
  frame_nv12->format = AV_PIX_FMT_NV12;
  frame_nv12->width = video_codec_ctx->width;
  frame_nv12->height = video_codec_ctx->height;
  av_frame_get_buffer(frame_nv12, 32);
  int32_t num_bytes = av_image_get_buffer_size(
      AV_PIX_FMT_NV12, video_codec_ctx->width, video_codec_ctx->height, 1);
  std::vector<uint8_t> buffer(num_bytes);
  av_image_fill_arrays(frame_nv12->data, frame_nv12->linesize, buffer.data(),
                       AV_PIX_FMT_NV12, video_codec_ctx->width,
                       video_codec_ctx->height, 1);

  // Audio part
  AVCodecContext* audio_codec_ctx = nullptr;
  SwrContext* swr_ctx = nullptr;
  if (audio_stream_index >= 0) {
    AVCodecParameters* audio_codecpar =
        fmt_ctx->streams[audio_stream_index]->codecpar;
    AVCodec* audio_codec = avcodec_find_decoder(audio_codecpar->codec_id);
    audio_codec_ctx = avcodec_alloc_context3(audio_codec);
    audio_codec_ctx->thread_type = FF_THREAD_SLICE;
    audio_codec_ctx->thread_count = 4;
    avcodec_parameters_to_context(audio_codec_ctx, audio_codecpar);
    avcodec_open2(audio_codec_ctx, audio_codec, nullptr);
    swr_ctx = swr_alloc_set_opts(
        nullptr, AV_CH_LAYOUT_MONO, AV_SAMPLE_FMT_FLT,
        SAMPLE_RATE,  // 输出单声道 float
        audio_codec_ctx->channel_layout
            ? audio_codec_ctx->channel_layout
            : av_get_default_channel_layout(audio_codec_ctx->channels),
        audio_codec_ctx->sample_fmt, audio_codec_ctx->sample_rate, 0, nullptr);
    swr_init(swr_ctx);
  }

  AVPacket* packet = av_packet_alloc();
  int64_t frame_count = 0;
  int32_t valid_frame_count = 0;
  std::vector<float> sensor_audio;
  while (av_read_frame(fmt_ctx, packet) >= 0) {
    if (packet->stream_index == video_stream_index) {
      if (avcodec_send_packet(video_codec_ctx, packet) == 0) {
        while (avcodec_receive_frame(video_codec_ctx, frame) == 0) {
          if (frame_count != std::round(valid_frame_count * (total_frames - 1) /
                                        static_cast<float>(nframes - 1))) {
            frame_count++;
            continue;
          }
          valid_frame_count++;
          sws_scale(sws_ctx, frame->data, frame->linesize, 0,
                    video_codec_ctx->height, frame_nv12->data,
                    frame_nv12->linesize);
          // 传入nv12数据
          if (demo_num == 1 || demo_num == 2) {
            omni_online_video_t video_input;
            video_input.width = frame_nv12->width;
            video_input.height = frame_nv12->height;
            video_input.y_ptr = frame_nv12->data[0];
            video_input.uv_ptr = frame_nv12->data[1];
            xlm_omni_feed_video_online(omni_handle, video_input);
          }
        }
      }
    } else if (packet->stream_index == audio_stream_index && audio_codec_ctx) {
      if (avcodec_send_packet(audio_codec_ctx, packet) == 0) {
        while (avcodec_receive_frame(audio_codec_ctx, frame) == 0) {
          int32_t dst_nb_samples = av_rescale_rnd(
              swr_get_delay(swr_ctx, audio_codec_ctx->sample_rate) +
                  frame->nb_samples,
              SAMPLE_RATE, audio_codec_ctx->sample_rate, AV_ROUND_UP);
          std::vector<float> buffer(dst_nb_samples);
          uint8_t* out_planes[1] = {reinterpret_cast<uint8_t*>(buffer.data())};
          int32_t converted =
              swr_convert(swr_ctx, out_planes, dst_nb_samples,
                          (const uint8_t**)frame->data, frame->nb_samples);
          sensor_audio.assign(buffer.begin(), buffer.begin() + converted);
          // 传入音频数据
          if (demo_num == 2 || demo_num == 3) {
            omni_online_audio_t audio_input;
            audio_input.data = sensor_audio.data();
            audio_input.data_size = sensor_audio.size();
            xlm_omni_feed_audio_online(omni_handle, audio_input);
          }
        }
      }
    }
    av_packet_unref(packet);
  }

  // Cleanup
  av_frame_free(&frame);
  av_frame_free(&frame_nv12);
  av_packet_free(&packet);
  avcodec_free_context(&video_codec_ctx);
  if (audio_codec_ctx) avcodec_free_context(&audio_codec_ctx);
  if (swr_ctx) swr_free(&swr_ctx);
  avformat_close_input(&fmt_ctx);
  sws_freeContext(sws_ctx);

  // 传入system_text
  if (demo_num == 1) {
    omni_online_text_t text_input;
    text_input.system_text =
        "You are Qwen, a virtual human developed by the Qwen Team, Alibaba "
        "Group, capable of perceiving auditory and visual inputs, as well as "
        "generating text and speech.";
    // 提供用户文本输入
    text_input.user_text = user_text.c_str();
    xlm_omni_feed_text_online(omni_handle, text_input);
  } else {
    omni_online_text_t text_input;
    text_input.system_text =
        "You are Qwen, a virtual human developed by the Qwen Team, Alibaba "
        "Group, capable of perceiving auditory and visual inputs, as well as "
        "generating text and speech.";
    text_input.user_text = nullptr;
    xlm_omni_feed_text_online(omni_handle, text_input);
  }

  // 调用omni处理流程
  xlm_omni(omni_handle, &input, nullptr);
}

void omni_conversation_once(xlm_handle_t omni_handle, int32_t bpu_core,
                            std::string video_path, std::string user_text) {
  xlm_input_s input;
  memset(&input, 0, sizeof(xlm_input_s));

  input.request_num = 1;
  std::vector<xlm_lm_request_t> requests(input.request_num);
  input.requests = requests.data();

  auto& request = input.requests[0];
  memset(&request, 0, sizeof(xlm_lm_request_t));

  request.type = XLM_INPUT_PROMPT;
  request.new_chat = true;
  request.system_prompt = nullptr;

  if (bpu_core == -1) {
    request.infer_backend = XLM_INFER_BACKEND_BPU_ANY;
  } else {
    request.infer_backend = kModelTypeMap.at(bpu_core);
  }

  std::cout << "板端Omni多模态大模型对话交互online demo\n"
            << "当前支持的多模态输入组合如下\n"
            << "1.从sensor获取视频(nv12)+文字\n"
            << "2.从sensor获取视频(nv12)+从sensor获取音频(pcm)\n"
            << "3.从sensor获取音频(pcm)\n"
            << "该demo以仿真形式模拟online场景，请输入1、2或3运行对应示例\n"
            << "退出请输入0" << std::endl;

  std::cout << "[User] <<< " << std::flush;
  int32_t demo_num = 0;
  while (std::cin >> demo_num) {
    if (demo_num == 0) {
      std::cout << "[system out] >>> 好的，祝您生活愉快，再见~" << std::endl;
      break;
    } else if (demo_num == 1 || demo_num == 2 || demo_num == 3) {
      online_demo(omni_handle, bpu_core, demo_num, video_path, user_text);
    } else {
      std::cout << "请按提示输入0~3中的数字并回车" << std::endl;
      std::cout << "[User] <<< " << std::flush;
    }
  }
}

int32_t main(int32_t argc, char** argv) {
  std::string config_path;

  // 解析命令行参数
  for (int32_t i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argc, argv);
      return 0;
    }
    if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      config_path = argv[++i];
    }
  }
  if (config_path.empty()) {
    std::cout << "Usage: " << argv[0] << " -c <config_file.json>" << std::endl;
    return 1;
  }

  // 打开并解析 JSON 文件
  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    std::cout << "failed to open json file: " << config_path << std::endl;
    return 1;
  }
  nlohmann::json config;
  try {
    config_file >> config;
  } catch (const std::exception& e) {
    std::cout << "error parsing json: " << e.what() << std::endl;
    return 1;
  }

  // 检查必填字段
  const std::vector<std::string> required_fields = {
      "visual_hbm_path", "audio_hbm_path", "text_hbm_path", "embed_tokens",
      "tokenizer_dir",   "model_type",     "online_mode",   "video_path"};
  for (const auto& field : required_fields) {
    if (!config.contains(field)) {
      std::cout << "missing required field: " << field << std::endl;
      return 1;
    }
  }

  // 必填字段赋值
  std::string visual_hbm_path = config["visual_hbm_path"];
  std::string audio_hbm_path = config["audio_hbm_path"];
  std::string text_hbm_path = config["text_hbm_path"];
  std::string embed_tokens = config["embed_tokens"];
  std::string tokenizer_dir = config["tokenizer_dir"];
  int32_t model_type = config["model_type"];
  bool online_mode = config["online_mode"].get<bool>();
  std::string video_path = config["video_path"];

  // 可选字段带默认值
  int32_t bpu_core = config.value("bpu_core", -1);
  std::string user_text = config.value("user_text", "请描述我在做什么");

  // 打印入参
  std::cout << "visual_hbm_path: " << visual_hbm_path << std::endl;
  std::cout << "audio_hbm_path: " << audio_hbm_path << std::endl;
  std::cout << "text_hbm_path: " << text_hbm_path << std::endl;
  std::cout << "embed_tokens: " << embed_tokens << std::endl;
  std::cout << "tokenizer_dir: " << tokenizer_dir << std::endl;
  std::cout << "model_type: " << model_type << std::endl;
  std::cout << "online_mode: " << online_mode << std::endl;
  std::cout << "bpu_core: " << bpu_core << std::endl;
  std::cout << "video_path: " << video_path << std::endl;
  std::cout << "user_text: " << user_text << std::endl;

  // 初始化参数
  xlm_common_params_t param = xlm_create_default_param();
  param.omni_visual_model_path = visual_hbm_path.c_str();
  param.omni_audio_model_path = audio_hbm_path.c_str();
  param.omni_text_model_path = text_hbm_path.c_str();
  param.embed_tokens = embed_tokens.c_str();
  param.token_config_path = tokenizer_dir.c_str();
  param.model_type = static_cast<xlm_model_type>(model_type);
  param.omni_online_mode = online_mode;  // online模式
  xlm_handle_t omni_handle = nullptr;
  int32_t ret = xlm_init(&param, callback, &omni_handle);
  if (ret == 0) {
    std::cout << "xlm init success" << std::endl;
  } else {
    std::cout << "xlm init failed" << std::endl;
    return 1;
  }

  if ((model_type == 5)) {
    omni_conversation_once(omni_handle, bpu_core, video_path, user_text);
  } else {
    std::cout << "this model type not support now, "
                 "only 5 can be used"
              << std::endl;
  }

  ret = xlm_destroy(&omni_handle);

  return 0;
}
