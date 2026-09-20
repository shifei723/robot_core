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
  if (state == XLM_STATE_ERROR) {
    std::cout << "run error" << std::endl;
  }
}

void print_usage(int32_t argc, char **argv) {
  std::cout << "useage:\n./oellm_ppl --config_path config_path" << std::endl;
}

void llm_ppl(xlm_handle_t llm_handle, std::string &hbm_path, int32_t bpu_core,
             std::string &ppl_testcase, bool &load_ckpt, int32_t text_data_num,
             int32_t max_length, int32_t stride) {
  xlm_input_s input;
  memset(&input, 0, sizeof(xlm_input_s));

  input.request_num = 1;
  std::vector<xlm_lm_request_t> requests(input.request_num);
  input.requests = requests.data();

  auto &request = input.requests[0];
  memset(&request, 0, sizeof(xlm_lm_request_t));

  request.type = XLM_INPUT_PROMPT;
  request.new_chat = true;
  request.system_prompt = nullptr;

  xlm_ppl_t ppl;
  memset(&ppl, 0, sizeof(xlm_ppl_t));
  request.ppl = &ppl;

  ppl.load_ckpt = load_ckpt;
  ppl.text_data_num = text_data_num;
  ppl.max_length = max_length;
  ppl.stride = stride;

  request.chat_template = nullptr;

  if (bpu_core == -1) {
    request.infer_backend = XLM_INFER_BACKEND_BPU_ANY;
  } else {
    request.infer_backend = kModelTypeMap.at(bpu_core);
  }

  std::ifstream ppl_bin_file(ppl_testcase, std::ios::binary);
  std::string ppl_bin_str((std::istreambuf_iterator<char>(ppl_bin_file)),
                          std::istreambuf_iterator<char>());
  std::cout << "开始测试文件: " << ppl_testcase << std::endl;
  request.prompt = ppl_bin_str.c_str();
  ppl.testcase_name = ppl_testcase.c_str();
  ppl.hbm_path = hbm_path.c_str();

  xlm_ppl(llm_handle, &input, nullptr);

  if (request.chat_template) {
    free(const_cast<char *>(request.chat_template));
    request.chat_template = nullptr;
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
    return -1;
  }

  // 打开并解析 JSON 文件
  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    std::cout << "failed to open json file: " << config_path << std::endl;
    return -1;
  }
  nlohmann::json config;
  try {
    config_file >> config;
  } catch (const std::exception &e) {
    std::cout << "error parsing json: " << e.what() << std::endl;
    return -1;
  }

  // 检查必填字段
  const std::vector<std::string> required_fields = {
      "hbm_path",     "tokenizer_dir", "model_type",
      "ppl_testcase", "max_length",    "stride"};
  for (const auto &field : required_fields) {
    if (!config.contains(field)) {
      std::cout << "missing required field: " << field << std::endl;
      return -1;
    }
  }

  // 必填字段赋值
  std::string hbm_path;
  std::string tokenizer_dir;
  std::string ppl_testcase;
  int32_t model_type;
  int32_t max_length;
  int32_t stride;
  std::string vlm_config_path;

  try {
    hbm_path = config["hbm_path"];
    tokenizer_dir = config["tokenizer_dir"];
    ppl_testcase = config["ppl_testcase"];
    model_type = config["model_type"];
    max_length = config["max_length"];
    stride = config["stride"];
  } catch (const std::exception &e) {
    std::cout << "error parsing required fields: " << e.what() << std::endl;
    return -1;
  }

  if (XLM_MODEL_TYPE_INTERNVL == model_type) {
    try {
      vlm_config_path = config["vlm_config_path"];
    } catch (const std::exception &e) {
      std::cout << "error parsing required fields: " << e.what() << std::endl;
      return -1;
    }
  }

  // 可选字段带默认值
  int32_t bpu_core = -1;
  int32_t text_data_num = 0;
  bool load_ckpt = false;

  // 安全解析可选字段，如果解析失败则使用默认值
  try {
    if (config.contains("bpu_core")) {
      bpu_core = config["bpu_core"];
    }
    if (config.contains("text_data_num")) {
      text_data_num = config["text_data_num"];
    }
    if (config.contains("load_ckpt")) {
      load_ckpt = config["load_ckpt"];
    }
  } catch (const std::exception &e) {
    std::cout << "warning: failed to parse some optional fields, using default "
                 "values: "
              << e.what() << std::endl;
  }

  // 打印入参
  std::cout << "hbm_path: " << hbm_path << std::endl;
  std::cout << "tokenizer_dir: " << tokenizer_dir << std::endl;
  std::cout << "ppl_testcase: " << ppl_testcase << std::endl;
  std::cout << "model_type: " << model_type << std::endl;
  std::cout << "max_length: " << max_length << std::endl;
  std::cout << "stride: " << stride << std::endl;
  std::cout << "bpu_core: " << bpu_core << std::endl;
  std::cout << "text_data_num: " << text_data_num << std::endl;
  std::cout << "load_ckpt: " << load_ckpt << std::endl;

  // 初始化参数
  xlm_common_params_t param = xlm_create_default_param();
  param.model_path = hbm_path.c_str();
  param.token_config_path = tokenizer_dir.c_str();
  param.model_type = static_cast<xlm_model_type>(model_type);
  param.k_cache_int8 = static_cast<bool>(false);
  param.config_path = vlm_config_path.c_str();
  xlm_handle_t llm_handle = nullptr;
  int32_t ret = xlm_init(&param, callback, &llm_handle);
  if (ret == 0) {
    std::cout << "xlm init success" << std::endl;
  } else {
    std::cout << "xlm init failed" << std::endl;
    return -1;
  }

  if ((model_type == XLM_MODEL_TYPE_DEEPSEEK) ||
      model_type == XLM_MODEL_TYPE_QWEN2_5 ||
      (model_type == XLM_MODEL_TYPE_INTERNVL) ||
      (model_type == XLM_MODEL_TYPE_INTERNLM)) {
    llm_ppl(llm_handle, hbm_path, bpu_core, ppl_testcase, load_ckpt,
            text_data_num, max_length, stride);
  } else {
    std::cout << "this model type not support now, "
                 "only 0/1/4/7 can be used"
              << std::endl;
    return -1;
  }

  ret = xlm_destroy(&llm_handle);

  return 0;
}
