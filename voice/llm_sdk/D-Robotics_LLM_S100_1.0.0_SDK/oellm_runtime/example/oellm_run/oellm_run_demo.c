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

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "xlm.h"  // NOLINT

typedef enum {
  PATH_TYPE_FILE,
  PATH_TYPE_DIRECTORY,
  PATH_TYPE_NONEXISTENT
} PathType;

typedef enum {
  PARSER_RESULT_SUCCESS,
  PARSER_RESULT_ERROR,
  PARSER_RESULT_HELP
} ParserResult;

typedef struct {
  int bpu_id;
  xlm_infer_backend backend;
} ModelTypeMapping;

static const ModelTypeMapping kModelTypeMap[4] = {
    {0, XLM_INFER_BACKEND_BPU_0},
    {1, XLM_INFER_BACKEND_BPU_1},
    {2, XLM_INFER_BACKEND_BPU_2},
    {3, XLM_INFER_BACKEND_BPU_3},
};

static const int kModelTypeMapSize = 4;

static PathType get_path_type(const char *path) {
  struct stat buffer;
  if (stat(path, &buffer) != 0) {
    return PATH_TYPE_NONEXISTENT;
  }
  if (S_ISREG(buffer.st_mode)) return PATH_TYPE_FILE;
  if (S_ISDIR(buffer.st_mode)) return PATH_TYPE_DIRECTORY;
  return PATH_TYPE_NONEXISTENT;
}

static xlm_infer_backend get_backend_from_core(int bpu_core) {
  for (int i = 0; i < kModelTypeMapSize; i++) {
    if (kModelTypeMap[i].bpu_id == bpu_core) {
      return kModelTypeMap[i].backend;
    }
  }
  return XLM_INFER_BACKEND_BPU_ANY;
}

static char *read_chat_template_file(const char *filepath) {
  FILE *file = fopen(filepath, "r");
  if (!file) {
    printf("open %s failed\n", filepath);
    return NULL;
  }

  if (fseek(file, 0, SEEK_END) != 0) {
    printf("seek end failed for %s\n", filepath);
    fclose(file);
    return NULL;
  }
  long tell_pos = ftell(file);
  if (tell_pos < 0) {
    printf("ftell failed for %s\n", filepath);
    fclose(file);
    return NULL;
  }
  if (tell_pos > LONG_MAX - 1) {
    printf("file too large %s\n", filepath);
    fclose(file);
    return NULL;
  }
  size_t filesize = tell_pos;
  if (fseek(file, 0, SEEK_SET) != 0) {
    printf("seek set failed for %s\n", filepath);
    fclose(file);
    return NULL;
  }
  char *content = (char *)malloc(filesize + 1);
  if (!content) {
    printf("malloc mem for content failed\n");
    fclose(file);
    return NULL;
  }

  size_t read_size = fread(content, 1, filesize, file);
  if (read_size != filesize) {
    printf("read file failed: %s\n", filepath);
    free(content);
    fclose(file);
    return NULL;
  }
  content[filesize] = '\0';

  fclose(file);
  return content;
}

static size_t valid_utf8_length(const char *str) {
  size_t i = 0;
  const size_t len = strlen(str);

  while (i < len) {
    const unsigned char c = (unsigned char)str[i];
    size_t char_len = 0;

    if (c <= 0x7F)
      char_len = 1;
    else if ((c & 0xE0) == 0xC0)
      char_len = 2;
    else if ((c & 0xF0) == 0xE0)
      char_len = 3;
    else if ((c & 0xF8) == 0xF0)
      char_len = 4;
    else
      break;

    if (i + char_len > len) break;

    bool valid_tail = true;
    for (size_t j = 1; j < char_len; j++) {
      if ((str[i + j] & 0xC0) != 0x80) {
        valid_tail = false;
        break;
      }
    }

    if (!valid_tail) break;
    i += char_len;
  }
  return i;
}

static void show_tips() {
  printf("板端大模型对话交互demo，请输入你的问题并按下回车 \n");
  printf("- 退出请输入exit\n");
  printf("- 清除缓存请输入reset\n");
  printf("[User] <<< ");
  fflush(stdout);
}

static void print_usage(void) {
  printf("Usage: oellm_run [options]\n\n");
  printf("Options:\n");
  printf("  --hbm_path <path>         Path to the HBM model file.\n");
  printf("  --tokenizer_dir <path>    Path to the tokenizer directory.\n");
  printf("  --config_path <path>      Path to the model config file.\n");
  printf("  --image_path <path>       Path to the input image file.\n");
  printf("  --template_path <path>    Path to the chat template file.\n");
  printf("  --model_type <int>        The type of the model. (Required)\n");
  printf("                            0: INTERNVL\n");
  printf("                            1: DEEPSEEK\n");
  printf("                            4: InternLM2\n");
  printf("                            7: QWEN2.5\n");
  printf(
      "  --bpu_core <int>          BPU core to use. [0, 1, 2, 3]. Default: any "
      "core.\n");
  printf("  -h, --help                Show this help message.\n");
}

static ParserResult validate_internvl_params(const char *config_path,
                                             const char *image_path) {
  if (!config_path || strlen(config_path) == 0 ||
      get_path_type(config_path) != PATH_TYPE_FILE) {
    fprintf(stderr,
            "Error: --config_path is required for INTERNVL model.Please check "
            "the path is correct or model type is correct.\n\n");
    return PARSER_RESULT_ERROR;
  }

  if (!image_path || strlen(image_path) == 0 ||
      get_path_type(image_path) != PATH_TYPE_FILE) {
    fprintf(stderr,
            "Error: --image_path is required for INTERNVL model. Please check "
            "the path is correct or model type is correct.\n\n");
    return PARSER_RESULT_ERROR;
  }

  return PARSER_RESULT_SUCCESS;
}

static ParserResult validate_deepseek_params(const char *hbm_path,
                                             const char *tokenizer_dir,
                                             const char *chat_template_path) {
  if (!hbm_path || strlen(hbm_path) == 0 ||
      get_path_type(hbm_path) != PATH_TYPE_FILE) {
    fprintf(stderr,
            "Error: --hbm_path is required for DeepSeek model. Please check "
            "the path is correct or model type is correct.\n\n");
    return PARSER_RESULT_ERROR;
  }

  if (!tokenizer_dir || strlen(tokenizer_dir) == 0 ||
      get_path_type(tokenizer_dir) != PATH_TYPE_DIRECTORY) {
    fprintf(stderr,
            "Error: --tokenizer_dir is required for DeepSeek model. Please "
            "check the path is correct or model type is correct.\n\n");
    return PARSER_RESULT_ERROR;
  }

  if (!chat_template_path || strlen(chat_template_path) == 0 ||
      get_path_type(chat_template_path) != PATH_TYPE_FILE) {
    fprintf(stderr,
            "Error: --template_path is required for DeepSeek model. Please "
            "check the path is correct or model type is correct.\n\n");
    return PARSER_RESULT_ERROR;
  }

  return PARSER_RESULT_SUCCESS;
}

static ParserResult validate_qwen_internlm_params(const char *hbm_path,
                                                  const char *tokenizer_dir,
                                                  const char *model_type_name) {
  if (!hbm_path || strlen(hbm_path) == 0 ||
      get_path_type(hbm_path) != PATH_TYPE_FILE) {
    fprintf(stderr,
            "Error: --hbm_path is required for %s model. Please check "
            "the path is correct or model type is correct.\n\n",
            model_type_name);
    return PARSER_RESULT_ERROR;
  }

  if (!tokenizer_dir || strlen(tokenizer_dir) == 0 ||
      get_path_type(tokenizer_dir) != PATH_TYPE_DIRECTORY) {
    fprintf(stderr,
            "Error: --tokenizer_dir is required for %s model. Please "
            "check the path is correct or model type is correct.\n\n",
            model_type_name);
    return PARSER_RESULT_ERROR;
  }

  return PARSER_RESULT_SUCCESS;
}

static ParserResult parse_arguments(int argc, char **argv, char **hbm_path,
                                    char **tokenizer_dir, char **image_path,
                                    char **config_path,
                                    char **chat_template_path, int *model_type,
                                    int *bpu_core) {
  // check if no arguments
  if (argc == 1) {
    return PARSER_RESULT_HELP;
  }

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      return PARSER_RESULT_HELP;
    }
  }

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--hbm_path") == 0) {
      if (i + 1 < argc) {
        *hbm_path = argv[++i];
      } else {
        return PARSER_RESULT_ERROR;
      }
    } else if (strcmp(argv[i], "--tokenizer_dir") == 0) {
      if (i + 1 < argc) {
        *tokenizer_dir = argv[++i];
      } else {
        return PARSER_RESULT_ERROR;
      }
    } else if (strcmp(argv[i], "--image_path") == 0) {
      if (i + 1 < argc) {
        *image_path = argv[++i];
      } else {
        return PARSER_RESULT_ERROR;
      }
    } else if (strcmp(argv[i], "--config_path") == 0) {
      if (i + 1 < argc) {
        *config_path = argv[++i];
      } else {
        return PARSER_RESULT_ERROR;
      }
    } else if (strcmp(argv[i], "--template_path") == 0) {
      if (i + 1 < argc) {
        *chat_template_path = argv[++i];
      } else {
        return PARSER_RESULT_ERROR;
      }
    } else if (strcmp(argv[i], "--model_type") == 0) {
      if (i + 1 < argc) {
        *model_type = atoi(argv[++i]);
      } else {
        return PARSER_RESULT_ERROR;
      }
    } else if (strcmp(argv[i], "--bpu_core") == 0) {
      if (i + 1 < argc) {
        *bpu_core = atoi(argv[++i]);
        if (*bpu_core < 0 || *bpu_core > 3) {
          fprintf(stderr, "Error: bpu_core must be in [0, 3]\n\n");
          return PARSER_RESULT_ERROR;
        }
      } else {
        return PARSER_RESULT_ERROR;
      }
    } else {
      fprintf(stderr, "Error: unknown argument: %s\n", argv[i]);
      return PARSER_RESULT_ERROR;
    }
  }

  if (*model_type != XLM_MODEL_TYPE_INTERNVL &&
      *model_type != XLM_MODEL_TYPE_DEEPSEEK &&
      *model_type != XLM_MODEL_TYPE_INTERNLM &&
      *model_type != XLM_MODEL_TYPE_QWEN2_5) {
    fprintf(stderr,
            "Error: --model_type is invalid. Must be 0 (INTERNVL) or 1 "
            "(DEEPSEEK) or 4(InternLM2) or 7(QWEN2.5).\n\n");
    return PARSER_RESULT_ERROR;
  }

  if (*model_type == XLM_MODEL_TYPE_INTERNVL) {
    return validate_internvl_params(*config_path, *image_path);
  } else if (*model_type == XLM_MODEL_TYPE_DEEPSEEK) {
    return validate_deepseek_params(*hbm_path, *tokenizer_dir,
                                    *chat_template_path);
  } else if (*model_type == XLM_MODEL_TYPE_QWEN2_5) {
    return validate_qwen_internlm_params(*hbm_path, *tokenizer_dir, "Qwen");
  } else if (*model_type == XLM_MODEL_TYPE_INTERNLM) {
    return validate_qwen_internlm_params(*hbm_path, *tokenizer_dir, "InternLM");
  } else {
    return PARSER_RESULT_ERROR;
  }
}

void callback(xlm_result_t *result, xlm_state_t state, void *userdata) {
  if (state == XLM_STATE_END) {
    printf("\n[User] <<< ");
    fflush(stdout);
  } else if (state == XLM_STATE_ERROR) {
    printf("run error\n");
  } else if (state == XLM_STATE_START) {
    printf("[Assistant] >>> %s", result->text);
    fflush(stdout);
  } else {
    printf("%s", result->text);
    fflush(stdout);
  }
}

void llm_infer(xlm_handle_t llm_handle, int bpu_core,
               const char *chat_template_path) {
  xlm_input_t input;
  memset(&input, 0, sizeof(xlm_input_t));

  input.request_num = 1;
  xlm_lm_request_t *requests =
      (xlm_lm_request_t *)malloc(sizeof(xlm_lm_request_t) * input.request_num);
  input.requests = requests;

  xlm_lm_request_t *request = &input.requests[0];
  memset(request, 0, sizeof(xlm_lm_request_t));

  request->type = XLM_INPUT_PROMPT;
  request->new_chat = 1;
  request->system_prompt = NULL;

  if (chat_template_path && strlen(chat_template_path) > 0) {
    request->chat_template = read_chat_template_file(chat_template_path);
  }

  if (bpu_core == -1) {
    request->infer_backend = XLM_INFER_BACKEND_BPU_ANY;
  } else {
    request->infer_backend = get_backend_from_core(bpu_core);
  }

  char input_str[10240];

  show_tips();

  while (fgets(input_str, sizeof(input_str), stdin)) {
    size_t len = strlen(input_str);
    if (len > 0 && input_str[len - 1] == '\n') {
      // remove the \n
      input_str[len - 1] = '\0';
    }

    size_t valid_len = valid_utf8_length(input_str);
    if (valid_len != strlen(input_str)) {
      input_str[valid_len] = '\0';
    }

    if (strcmp(input_str, "exit") == 0) {
      printf("[system out] >>> 好的，祝您生活愉快，再见~\n");
      break;
    } else if (strcmp(input_str, "reset") == 0) {
      request->new_chat = true;
      printf("[User] <<< ");
      fflush(stdout);
      continue;
    }

    request->prompt = input_str;
    xlm_infer_async(llm_handle, &input, NULL);
    request->new_chat = true;
  }

  if (request->chat_template) {
    free((void *)request->chat_template);
    request->chat_template = NULL;
  }
  free(requests);
}

void vlm_infer(xlm_handle_t llm_handle, const char *image_path, int bpu_core) {
  if (!image_path || strlen(image_path) == 0) {
    fprintf(stderr, "image_path is empty\n");
    return;
  }

  xlm_input_t input;
  memset(&input, 0, sizeof(xlm_input_t));

  input.request_num = 1;
  xlm_lm_request_t *requests =
      (xlm_lm_request_t *)malloc(sizeof(xlm_lm_request_t) * input.request_num);
  input.requests = requests;

  xlm_lm_request_t *request = &input.requests[0];
  memset(request, 0, sizeof(xlm_lm_request_t));

  request->new_chat = true;
  if (bpu_core == -1) {
    request->infer_backend = XLM_INFER_BACKEND_BPU_ANY;
  } else {
    request->infer_backend = get_backend_from_core(bpu_core);
  }

  request->type = XLM_INPUT_MULTI_MODAL;
  request->multi_modal_requset.image_num = 1;
  xlm_input_image_t *images = (xlm_input_image_t *)malloc(
      sizeof(xlm_input_image_t) * request->multi_modal_requset.image_num);
  request->multi_modal_requset.images = images;

  xlm_input_image_t *image = &request->multi_modal_requset.images[0];
  memset(image, 0, sizeof(xlm_input_image_t));
  image->image_path = image_path;

  show_tips();

  char input_str[2048];
  // vlm only support chat once
  fgets(input_str, sizeof(input_str), stdin);
  size_t len = strlen(input_str);
  if (len > 0 && input_str[len - 1] == '\n') {
    input_str[len - 1] = '\0';
  }

  size_t valid_len = valid_utf8_length(input_str);
  if (valid_len != strlen(input_str)) {
    input_str[valid_len] = '\0';
  }

  request->multi_modal_requset.prompt = input_str;
  xlm_infer_async(llm_handle, &input, NULL);

  free(images);
  free(requests);
}

int main(int argc, char **argv) {
  char *hbm_path = NULL;
  char *tokenizer_dir = NULL;
  char *image_path = NULL;
  char *config_path = NULL;
  char *chat_template_path = NULL;
  int model_type = -1;
  int bpu_core = -1;

  ParserResult parse_result = parse_arguments(
      argc, argv, &hbm_path, &tokenizer_dir, &image_path, &config_path,
      &chat_template_path, &model_type, &bpu_core);
  if (parse_result == PARSER_RESULT_HELP) {
    print_usage();
    return 0;
  } else if (parse_result == PARSER_RESULT_ERROR) {
    print_usage();
    return 1;
  }

  xlm_common_params_t param = xlm_create_default_param();
  param.sampling.min_keep = 1;
  param.sampling.min_p = 0.0f;
  param.sampling.temp = 1.0f;
  param.sampling.top_k = 50;
  param.sampling.top_p = 1.00f;
  param.sampling.typ_p = 1.00f;
  param.sampling.penalty_last_n = 128;
  param.sampling.penalty_freq = 0.1;
  param.sampling.penalty_present = 0.1;
  param.sampling.penalty_repeat = 1.2;

  param.model_path = hbm_path;
  param.token_config_path = tokenizer_dir;
  param.config_path = config_path;
  param.model_type = (xlm_model_type)model_type;
  if (model_type == XLM_MODEL_TYPE_INTERNLM) param.k_cache_int8 = true;

  xlm_handle_t llm_handle = NULL;
  int ret = xlm_init(&param, callback, &llm_handle);
  if (ret == 0) {
    printf("xlm init success\n");
  } else {
    printf("xlm init failed\n");
    return 1;
  }

  if (model_type == XLM_MODEL_TYPE_INTERNVL) {
    vlm_infer(llm_handle, image_path, bpu_core);
  } else {
    llm_infer(llm_handle, bpu_core, chat_template_path);
  }

  ret = xlm_destroy(&llm_handle);

  return 0;
}
