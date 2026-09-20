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
#include <limits>
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

static char *read_chat_template_file(const char *filepath) {
  FILE *file = fopen(filepath, "r");
  if (!file) {
    std::cout << "open " << filepath << " failed" << std::endl;
    return nullptr;
  }

  if (fseek(file, 0, SEEK_END) != 0) {
    std::cout << "seek end failed for " << filepath << std::endl;
    fclose(file);
    return nullptr;
  }
  long tell_pos = ftell(file);
  if (tell_pos < 0) {
    std::cout << "ftell failed for " << filepath << std::endl;
    fclose(file);
    return nullptr;
  }
  if (static_cast<unsigned long>(tell_pos) >
      std::numeric_limits<size_t>::max() - 1) {
    std::cout << "file too large: " << filepath << std::endl;
    fclose(file);
    return nullptr;
  }
  size_t filesize = static_cast<size_t>(tell_pos);
  if (fseek(file, 0, SEEK_SET) != 0) {
    std::cout << "seek set failed for " << filepath << std::endl;
    fclose(file);
    return nullptr;
  }
  char *content = reinterpret_cast<char *>(malloc(filesize + 1));
  if (!content) {
    std::cout << "malloc mem for content failed" << std::endl;
    fclose(file);
    return nullptr;
  }

  size_t read_size = fread(content, 1, filesize, file);
  if (read_size != filesize) {
    std::cout << "read file failed: " << filepath << std::endl;
    free(content);
    fclose(file);
    return nullptr;
  }
  content[filesize] = '\0';

  fclose(file);
  return content;
}

struct ChatContext {
  std::string current_output;
  std::string all_messages;
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
  std::cout << "useage:\n./oellm_multichat --config_path config_path"
            << std::endl;
}

void llm_multichat(xlm_handle_t llm_handle, int32_t bpu_core,
                   std::string &chat_template_path) {
  int32_t batch_num = 1;

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

  if (!chat_template_path.empty()) {
    request.chat_template = read_chat_template_file(chat_template_path.c_str());
  }

  if (request.chat_template == nullptr){
    return;
  } 

  if (bpu_core == -1) {
    request.infer_backend = XLM_INFER_BACKEND_BPU_ANY;
  } else {
    request.infer_backend = kModelTypeMap.at(bpu_core);
  }

  std::cout << "板端大模型多轮对话交互demo，请输入你的问题并按下回车 \n"
            << "- 退出请输入exit\n"
            << "- 清除缓存请输入reset" << std::endl;

  std::cout << "[User] <<< " << std::flush;

  std::string input_str;
  // static ChatContext ctx;

  while (std::getline(std::cin, input_str)) {
    if (input_str == "exit") {
      std::cout << "[system out] >>> 好的，祝您生活愉快，再见~" << std::endl;
      break;
    } else if (input_str == "reset") {
      request.new_chat = true;
      std::cout << "[User] <<< " << std::flush;
      continue;
    }

    request.prompt = (input_str).c_str();
    xlm_infer(llm_handle, &input, nullptr);

    request.new_chat = false;
  }

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
  const std::vector<std::string> required_fields = {"hbm_path", "tokenizer_dir",
                                                    "model_type"};
  for (const auto &field : required_fields) {
    if (!config.contains(field)) {
      std::cout << "missing required field: " << field << std::endl;
      return 1;
    }
  }

  if (!config.contains("template_path")) {
    std::cout << "missing required field: template_path" << std::endl;
    return 1;
  }

  // 必填字段赋值
  std::string hbm_path;
  std::string tokenizer_dir;
  std::string template_path;
  int32_t model_type = 0;
  int32_t bpu_core = -1;

  try {
    // 1. 验证字符串类型字段（hbm_path/tokenizer_dir/template_path）
    if (!config["hbm_path"].is_string()) {
      throw std::runtime_error("field 'hbm_path' must be string type");
    }
    if (!config["tokenizer_dir"].is_string()) {
      throw std::runtime_error("field 'tokenizer_dir' must be string type");
    }
    if (!config["template_path"].is_string()) {
      throw std::runtime_error("field 'template_path' must be string type");
    }
    // 2. 验证数字类型字段（model_type）
    if (!config["model_type"]
             .is_number_integer()) {  // 确保是整数（排除浮点数）
      throw std::runtime_error(
          "field 'model_type' must be integer type (e.g. 1)");
    }

    // 类型验证通过后，安全提取值
    hbm_path =
        config["hbm_path"].get<std::string>();  // 显式 get<string> 更安全
    tokenizer_dir = config["tokenizer_dir"].get<std::string>();
    template_path = config["template_path"];
    model_type = config["model_type"].get<int32_t>();  // 显式 get<int32_t>

    // 可选字段：同样用 try-catch 保护，验证类型
    if (config.contains("bpu_core")) {
      if (!config["bpu_core"].is_number_integer()) {
        throw std::runtime_error(
            "field 'bpu_core' must be integer type (e.g. -1, 0)");
      }
      bpu_core = config["bpu_core"].get<int32_t>();
    } else {
      bpu_core = -1;  // 默认值（与原有逻辑一致）
    }

  } catch (const nlohmann::json::type_error &e) {
    // 捕获 nlohmann::json 类型错误（如字段类型不匹配）
    std::cout << "json type error: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    // 捕获自定义类型验证错误和其他通用异常
    std::cout << "invalid config file: " << e.what() << std::endl;
    return 1;
  }

  // 打印入参
  std::cout << "hbm_path: " << hbm_path << std::endl;
  std::cout << "tokenizer_dir: " << tokenizer_dir << std::endl;
  std::cout << "template_path: " << template_path << std::endl;
  std::cout << "model_type: " << model_type << std::endl;
  std::cout << "bpu_core: " << bpu_core << std::endl;

  // 初始化参数
  xlm_common_params_t param = xlm_create_default_param();
  param.model_path = hbm_path.c_str();
  param.token_config_path = tokenizer_dir.c_str();
  param.model_type = static_cast<xlm_model_type>(model_type);
  param.k_cache_int8 = static_cast<bool>(false);
  xlm_handle_t llm_handle = nullptr;

  int32_t ret = xlm_init(&param, callback, &llm_handle);
  if (ret == 0) {
    std::cout << "xlm init success" << std::endl;
  } else {
    std::cout << "xlm init failed" << std::endl;
    return 1;
  }

  if (model_type == XLM_MODEL_TYPE_DEEPSEEK ||
      model_type == XLM_MODEL_TYPE_QWEN2_5) {
    llm_multichat(llm_handle, bpu_core, template_path);
  } else {
    std::cout << "this model type not support now, "
                 "only 1,7 can be used"
              << std::endl;
    return 1;
  }

  ret = xlm_destroy(&llm_handle);

  return 0;
}
