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

#include <cstring>
#include <filesystem>  // NOLINT
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "xlm.h"  // NOLINT

const std::unordered_map<int32_t, xlm_infer_backend> kModelTypeMap = {
    {0, XLM_INFER_BACKEND_BPU_0},
    {1, XLM_INFER_BACKEND_BPU_1},
    {2, XLM_INFER_BACKEND_BPU_2},
    {3, XLM_INFER_BACKEND_BPU_3},
};

void callback(xlm_result_s *result, xlm_state_e state, void *userdata) {
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
  std::cout << "useage:\n./oellm_omni_offline --config_path config_path"
            << std::endl;
}

void omni_conversation_once(xlm_handle_t omni_handle, int32_t bpu_core) {
  xlm_input_s input;
  memset(&input, 0, sizeof(xlm_input_s));

  input.request_num = 1;
  std::vector<xlm_lm_request_t> requests(input.request_num);
  input.requests = requests.data();

  auto &request = input.requests[0];
  memset(&request, 0, sizeof(xlm_lm_request_t));

  request.type = XLM_INPUT_PROMPT;
  request.system_prompt = nullptr;

  if (bpu_core == -1) {
    request.infer_backend = XLM_INFER_BACKEND_BPU_ANY;
  } else {
    request.infer_backend = kModelTypeMap.at(bpu_core);
  }

  std::cout
      << "板端Omni多模态大模型对话交互offline "
         "demo，请输入包含输入信息的json文件路径\n"
      << "当前支持的多模态输入组合如下\n"
      << "1.文字\n"
      << "2.音频(mp3|wav)\n"
      << "3.图片(jpg|png|bmp|jpeg)\n"
      << "4.图片(jpg|png|bmp|jpeg)+文字\n"
      << "5.图片(jpg|png|bmp|jpeg)+音频(mp3|wav)\n"
      << "6.含音频的视频(mp4|mkv)+文字\n"
      << "7.含音频的视频(mp4|mkv)+音频(mp3|wav)\n"
      << "8.不含音频的视频(mp4|mkv)\n"
      << "9.不含音频的视频(mp4|mkv)+文字\n"
      << "json内容填写示例\n"
      << R"({"conversation": [{"role": "system","content": [{"type": "text","text": "You are Qwen, a virtual human developed by the Qwen Team, Alibaba Group, capable of perceiving auditory and visual inputs, as well as generating text and speech."}]},{"role": "user","content": [{"type": "text","text": "简单介绍人工智能"}]}]})"
      << "\n"
      << "用户输入示例\n"
      << "[User] <<< ./omni_offline_prompt.json\n"
      << "退出请输入exit" << std::endl;

  std::cout << "[User] <<< " << std::flush;
  std::string input_prompt;
  while (std::getline(std::cin, input_prompt)) {
    if (input_prompt == "exit") {
      std::cout << "[system out] >>> 好的，祝您生活愉快，再见~" << std::endl;
      break;
    }
    request.prompt_json = input_prompt.c_str();
    if (xlm_omni(omni_handle, &input, nullptr)!=0){
      // 刷新LOGE缓冲区
      fflush(stderr);
      fflush(stdout);
      std::cout<< std::flush << "[User] <<< ";
    }
  }
}

int32_t main(int32_t argc, char **argv) {
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
  } catch (const std::exception &e) {
    std::cout << "error parsing json: " << e.what() << std::endl;
    return 1;
  }

  // 检查必填字段
  const std::vector<std::string> required_fields = {
      "visual_hbm_path", "audio_hbm_path", "text_hbm_path", "embed_tokens",
      "tokenizer_dir",   "model_type",     "online_mode"};
  for (const auto &field : required_fields) {
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

  // 可选字段带默认值
  int32_t bpu_core = config.value("bpu_core", -1);

  // 打印入参
  std::cout << "visual_hbm_path: " << visual_hbm_path << std::endl;
  std::cout << "audio_hbm_path: " << audio_hbm_path << std::endl;
  std::cout << "text_hbm_path: " << text_hbm_path << std::endl;
  std::cout << "embed_tokens: " << embed_tokens << std::endl;
  std::cout << "tokenizer_dir: " << tokenizer_dir << std::endl;
  std::cout << "model_type: " << model_type << std::endl;
  std::cout << "online_mode: " << online_mode << std::endl;
  std::cout << "bpu_core: " << bpu_core << std::endl;

  // 初始化参数
  xlm_common_params_t param = xlm_create_default_param();
  param.omni_visual_model_path = visual_hbm_path.c_str();
  param.omni_audio_model_path = audio_hbm_path.c_str();
  param.omni_text_model_path = text_hbm_path.c_str();
  param.embed_tokens = embed_tokens.c_str();
  param.token_config_path = tokenizer_dir.c_str();
  param.model_type = static_cast<xlm_model_type>(model_type);
  param.omni_online_mode = online_mode;  // offline模式
  xlm_handle_t omni_handle = nullptr;
  int32_t ret = xlm_init(&param, callback, &omni_handle);
  if (ret == 0) {
    std::cout << "xlm init success" << std::endl;
  } else {
    std::cout << "xlm init failed" << std::endl;
    return 1;
  }

  if ((model_type == 5)) {
    omni_conversation_once(omni_handle, bpu_core);
  } else {
    std::cout << "this model type not support now, "
                 "only 5 can be used"
              << std::endl;
  }

  ret = xlm_destroy(&omni_handle);

  return 0;
}
