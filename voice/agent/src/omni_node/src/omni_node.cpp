#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <queue>

extern "C" {
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "xlm.h"

using namespace std::chrono_literals;

namespace {
constexpr int32_t kSampleRate = 16000;
constexpr int32_t kMinImageRepeat = 2;

uint16_t ReadLe16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t ReadLe32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0] | (static_cast<uint32_t>(p[1]) << 8) |
                               (static_cast<uint32_t>(p[2]) << 16) |
                               (static_cast<uint32_t>(p[3]) << 24));
}

bool GetFfmpegFormatAndStep(const std::string& enc, int width,
                            AVPixelFormat& pix_fmt, size_t& line_step) {
  if (enc == "bgr8") {
    pix_fmt = AV_PIX_FMT_BGR24;
    line_step = static_cast<size_t>(width) * 3;
    return true;
  }
  if (enc == "rgb8") {
    pix_fmt = AV_PIX_FMT_RGB24;
    line_step = static_cast<size_t>(width) * 3;
    return true;
  }
  if (enc == "mono8") {
    pix_fmt = AV_PIX_FMT_GRAY8;
    line_step = static_cast<size_t>(width);
    return true;
  }
  if (enc == "nv12" || enc == "NV12") {
    pix_fmt = AV_PIX_FMT_NV12;
    line_step = static_cast<size_t>(width);
    return true;
  }
  return false;
}

const std::unordered_map<int32_t, xlm_infer_backend> kBpuCoreMap = {
    {0, XLM_INFER_BACKEND_BPU_0},
    {1, XLM_INFER_BACKEND_BPU_1},
    {2, XLM_INFER_BACKEND_BPU_2},
    {3, XLM_INFER_BACKEND_BPU_3},
};
}  // namespace

struct Nv12Frame {
  std::vector<uint8_t> data;
  int32_t width{0};
  int32_t height{0};
};

struct InferRequest {
  uint64_t req_id{0};
  std::vector<uint8_t> wav_bytes;
  sensor_msgs::msg::Image image;
};

struct CallbackCtx {
  class OmniMultimodalNode* node;
  uint64_t req_id;
  std::string buffer;
};

class OmniMultimodalNode : public rclcpp::Node {
 public:
  OmniMultimodalNode() : Node("omni_multimodal_node") {
    RCLCPP_INFO(this->get_logger(), "===== Omni Multimodal Node Start =====");

    DeclareParams();
    GetParams();

    text_pub_ = this->create_publisher<std_msgs::msg::String>(text_topic_, 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>(state_topic_, 10);

    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, img_qos,
        std::bind(&OmniMultimodalNode::OnImageRecv, this, std::placeholders::_1));

    audio_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        audio_topic_, 10,
        std::bind(&OmniMultimodalNode::OnAudioRecv, this, std::placeholders::_1));

    timeout_timer_ = this->create_wall_timer(
        50ms, std::bind(&OmniMultimodalNode::CheckPairTimeout, this));

    worker_thread_ = std::thread(&OmniMultimodalNode::InferWorker, this);

    if (!InitXlm()) {
      RCLCPP_ERROR(this->get_logger(), "XLM init failed!");
    } else {
      RCLCPP_INFO(this->get_logger(), "XLM init success, ready for inference");
    }
  }

  ~OmniMultimodalNode() override {
    {
      std::lock_guard<std::mutex> lk(req_mtx_);
      stop_worker_ = true;
    }
    req_cv_.notify_one();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    if (xlm_handle_ != nullptr) {
      xlm_destroy(&xlm_handle_);
      RCLCPP_INFO(this->get_logger(), "XLM resource released");
    }
    RCLCPP_INFO(this->get_logger(), "===== Omni Multimodal Node Exit =====");
  }

 private:
  void DeclareParams() {
    this->declare_parameter<std::string>("omni_visual_hbm_path", "");
    this->declare_parameter<std::string>("omni_audio_hbm_path", "");
    this->declare_parameter<std::string>("omni_text_hbm_path", "");
    this->declare_parameter<std::string>("omni_embed_tokens", "");
    this->declare_parameter<std::string>("omni_tokenizer_dir", "");

    this->declare_parameter<int>("omni_model_type", 5);
    this->declare_parameter<bool>("omni_online_mode", true);
    this->declare_parameter<int>("omni_bpu_core", -1);
    this->declare_parameter<int>("omni_image_repeat", kMinImageRepeat);

    this->declare_parameter<std::string>("omni_audio_topic", "/asr/wav_audio");
    this->declare_parameter<std::string>("omni_image_topic", "/vision/image");
    this->declare_parameter<std::string>("omni_text_topic", "/omni/output_text");
    this->declare_parameter<std::string>("omni_state_topic", "/omni/node_state");

    this->declare_parameter<double>("omni_wait_timeout", 0.5);
    this->declare_parameter<std::string>("omni_punctuation", "。！？；，,.!?;:");
    this->declare_parameter<std::string>(
        "omni_system_prompt",
        "你是小力，一个智能助手，可以理解图像和音频并回答问题。");
  }

  void GetParams() {
    visual_hbm_ = this->get_parameter("omni_visual_hbm_path").as_string();
    audio_hbm_ = this->get_parameter("omni_audio_hbm_path").as_string();
    text_hbm_ = this->get_parameter("omni_text_hbm_path").as_string();
    embed_tokens_ = this->get_parameter("omni_embed_tokens").as_string();
    tokenizer_dir_ = this->get_parameter("omni_tokenizer_dir").as_string();

    model_type_ = this->get_parameter("omni_model_type").as_int();
    online_mode_ = this->get_parameter("omni_online_mode").as_bool();
    bpu_core_idx_ = this->get_parameter("omni_bpu_core").as_int();
    image_repeat_ = this->get_parameter("omni_image_repeat").as_int();
    if (image_repeat_ < 1) image_repeat_ = 1;

    audio_topic_ = this->get_parameter("omni_audio_topic").as_string();
    image_topic_ = this->get_parameter("omni_image_topic").as_string();
    text_topic_ = this->get_parameter("omni_text_topic").as_string();
    state_topic_ = this->get_parameter("omni_state_topic").as_string();

    wait_timeout_s_ = this->get_parameter("omni_wait_timeout").as_double();
    punctuation_ = this->get_parameter("omni_punctuation").as_string();
    system_prompt_ = this->get_parameter("omni_system_prompt").as_string();
  }

  // XLM 全局回调桥接函数（静态，符合C接口要求）
  static void XlmCallbackBridge(xlm_result_t* result, xlm_state_t state, void* userdata) {
    auto* ctx = static_cast<CallbackCtx*>(userdata);
    if (!ctx || !ctx->node) return;
    ctx->node->OnXlmCallback(result, state, ctx);
  }

  bool InitXlm() {
    if (visual_hbm_.empty() || audio_hbm_.empty() || text_hbm_.empty() ||
        embed_tokens_.empty() || tokenizer_dir_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Model path parameters are incomplete");
      return false;
    }

    xlm_common_params_t params = xlm_create_default_param();
    // 完全对齐头文件结构体字段
    params.omni_visual_model_path = visual_hbm_.c_str();
    params.omni_audio_model_path = audio_hbm_.c_str();
    params.omni_text_model_path = text_hbm_.c_str();
    params.embed_tokens = embed_tokens_.c_str();
    params.token_config_path = tokenizer_dir_.c_str();
    params.model_type = static_cast<xlm_model_type>(model_type_);
    params.omni_online_mode = online_mode_;

    // 对齐头文件函数签名
    int ret = xlm_init(&params, &OmniMultimodalNode::XlmCallbackBridge,
                       reinterpret_cast<void**>(&xlm_handle_));
    if (ret != 0 || xlm_handle_ == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "xlm_init failed, ret: %d", ret);
      return false;
    }
    return true;
  }

  void OnAudioRecv(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(pair_mtx_);
    uint64_t new_id = ++global_req_id_;
    waiting_wav_ = msg->data;
    waiting_req_id_ = new_id;
    wait_deadline_ = this->now() + rclcpp::Duration::from_seconds(wait_timeout_s_);
    waiting_image_ = false;

    PublishState("Audio received, waiting for image");
  }

  void OnImageRecv(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(pair_mtx_);
    if (waiting_req_id_ == 0 || waiting_wav_.empty()) {
      return;
    }

    InferRequest req;
    req.req_id = waiting_req_id_;
    req.wav_bytes.swap(waiting_wav_);
    req.image = *msg;

    {
      std::lock_guard<std::mutex> req_lk(req_mtx_);
      infer_queue_.push(std::move(req));
    }
    req_cv_.notify_one();

    waiting_req_id_ = 0;
    waiting_wav_.clear();
    waiting_image_ = true;

    PublishState("Image matched, push to infer queue");
  }

  void CheckPairTimeout() {
    std::lock_guard<std::mutex> lk(pair_mtx_);
    if (waiting_req_id_ == 0) return;
    if (this->now() > wait_deadline_) {
      waiting_req_id_ = 0;
      waiting_wav_.clear();
      waiting_image_ = false;
      PublishState("Audio wait image timeout, discard");
    }
  }

  void InferWorker() {
    while (!stop_worker_) {
      std::optional<InferRequest> current_req;
      {
        std::unique_lock<std::mutex> lk(req_mtx_);
        req_cv_.wait(lk, [this]() {
          return stop_worker_ || !infer_queue_.empty();
        });
        if (stop_worker_) break;
        if (!infer_queue_.empty()) {
          current_req = std::move(infer_queue_.front());
          infer_queue_.pop();
        }
      }

      if (!current_req.has_value()) continue;
      RunInfer(current_req.value());
    }
  }

  void RunInfer(const InferRequest& req) {
    std::vector<float> audio_pcm;
    if (!DecodeWavToPcm(req.wav_bytes, audio_pcm)) {
      PublishState("Decode WAV failed");
      return;
    }

    Nv12Frame nv12_frame;
    if (!ConvertImageToNv12(req.image, nv12_frame)) {
      PublishState("Convert image to NV12 failed");
      return;
    }

    // 构造推理输入请求，完全对齐头文件结构体
    xlm_input_t input;
    std::memset(&input, 0, sizeof(input));
    input.request_num = 1;
    std::vector<xlm_lm_request_t> requests(input.request_num);
    input.requests = requests.data();

    auto& lm_req = input.requests[0];
    std::memset(&lm_req, 0, sizeof(lm_req));
    lm_req.type = XLM_INPUT_PROMPT;
    lm_req.new_chat = true;
    lm_req.system_prompt = nullptr;

    if (bpu_core_idx_ == -1) {
      lm_req.infer_backend = XLM_INFER_BACKEND_BPU_ANY;
    } else {
      auto it = kBpuCoreMap.find(bpu_core_idx_);
      lm_req.infer_backend = (it == kBpuCoreMap.end())
                                  ? XLM_INFER_BACKEND_BPU_ANY
                                  : it->second;
    }

    // 1. 喂入视频帧
    omni_online_video_t video_input;
    size_t y_size = static_cast<size_t>(nv12_frame.width) * nv12_frame.height;
    video_input.y_ptr = nv12_frame.data.data();
    video_input.uv_ptr = nv12_frame.data.data() + y_size;
    video_input.width = nv12_frame.width;
    video_input.height = nv12_frame.height;

    for (int i = 0; i < image_repeat_; ++i) {
      int ret = xlm_omni_feed_video_online(xlm_handle_, video_input);
      if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "feed video failed, ret: %d", ret);
        PublishState("feed video failed");
        return;
      }
    }

    // 2. 喂入音频
    omni_online_audio_t audio_input;
    audio_input.data = audio_pcm.data();
    audio_input.data_size = static_cast<int32_t>(audio_pcm.size());
    int ret = xlm_omni_feed_audio_online(xlm_handle_, audio_input);
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "feed audio failed, ret: %d", ret);
      PublishState("feed audio failed");
      return;
    }

    // 3. 喂入文本提示词
    omni_online_text_t text_input;
    text_input.system_text = system_prompt_.c_str();
    text_input.user_text = nullptr;
    ret = xlm_omni_feed_text_online(xlm_handle_, text_input);
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "feed text failed, ret: %d", ret);
      PublishState("feed text failed");
      return;
    }

    // 回调上下文
    CallbackCtx ctx;
    ctx.node = this;
    ctx.req_id = req.req_id;
    ctx.buffer.clear();

    PublishState("running inference");
    // 执行多模态推理，第三个参数为回调userdata
    ret = xlm_omni(xlm_handle_, &input, &ctx);
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "xlm_omni execute failed, ret: %d", ret);
      PublishState("infer execute failed");
      return;
    }
    PublishState("infer complete");
  }

  // XLM 流式回调处理
  void OnXlmCallback(xlm_result_t* result, xlm_state_t state, CallbackCtx* ctx) {
    if (state == XLM_STATE_ERROR) {
      PublishState("infer callback error");
      return;
    }

    std::string piece;
    if (result != nullptr && result->text != nullptr) {
      piece = result->text;
    }

    if (state == XLM_STATE_START) {
      ctx->buffer.clear();
    }

    // 修复：XLM_STATE_NORMAL -> XLM_STATE_RUNNING，对齐头文件枚举
    if (!piece.empty() && (state == XLM_STATE_START || state == XLM_STATE_RUNNING)) {
      ctx->buffer += piece;
      if (CheckEndWithPunct(ctx->buffer, punctuation_)) {
        PublishText(ctx->req_id, ctx->buffer);
        ctx->buffer.clear();
      }
    }

    if (state == XLM_STATE_END) {
      ctx->buffer += piece;
      if (!ctx->buffer.empty()) {
        PublishText(ctx->req_id, ctx->buffer);
      }
      PublishState("Infer finished");
    }
  }

  bool DecodeWavToPcm(const std::vector<uint8_t>& wav, std::vector<float>& out_pcm) {
    if (wav.size() < 44) return false;
    const uint8_t* ptr = wav.data();

    if (std::memcmp(ptr, "RIFF", 4) != 0) return false;
    if (std::memcmp(ptr + 8, "WAVE", 4) != 0) return false;

    uint16_t channels = ReadLe16(ptr + 22);
    uint32_t sample_rate = ReadLe32(ptr + 24);
    uint16_t bit_depth = ReadLe16(ptr + 34);
    uint32_t data_len = ReadLe32(ptr + 40);
    const uint8_t* data_ptr = ptr + 44;

    if (sample_rate != kSampleRate || channels != 1 || bit_depth != 16) {
      RCLCPP_WARN(this->get_logger(),
                  "Only support 16k/1ch/16bit wav, current: %u ch:%u bit:%u",
                  sample_rate, channels, bit_depth);
      return false;
    }

    size_t sample_cnt = data_len / 2;
    out_pcm.resize(sample_cnt);
    for (size_t i = 0; i < sample_cnt; ++i) {
      int16_t val = static_cast<int16_t>(ReadLe16(data_ptr + i * 2));
      out_pcm[i] = static_cast<float>(val) / 32768.0f;
    }
    return true;
  }

  bool ConvertImageToNv12(const sensor_msgs::msg::Image& msg, Nv12Frame& out) {
    int w = static_cast<int>(msg.width);
    int h = static_cast<int>(msg.height);
    if (w <= 0 || h <= 0) return false;
    if ((w % 2) != 0 || (h % 2) != 0) {
      RCLCPP_WARN(this->get_logger(), "NV12 require even width & height");
      return false;
    }

    AVPixelFormat src_fmt;
    size_t src_line;
    if (!GetFfmpegFormatAndStep(msg.encoding, w, src_fmt, src_line)) {
      RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s",
                   msg.encoding.c_str());
      return false;
    }

    size_t total_size = static_cast<size_t>(w) * h * 3 / 2;
    out.data.resize(total_size);
    out.width = w;
    out.height = h;

    uint8_t* dst_data[2] = {out.data.data(), out.data.data() + static_cast<size_t>(w) * h};
    int dst_linesize[2] = {w, w};

    const uint8_t* src_data[1] = {msg.data.data()};
    int src_linesize[1] = {static_cast<int>(src_line)};

    SwsContext* sws_ctx = sws_getContext(w, h, src_fmt, w, h, AV_PIX_FMT_NV12,
                                         SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws_ctx) return false;

    sws_scale(sws_ctx, src_data, src_linesize, 0, h, dst_data, dst_linesize);
    sws_freeContext(sws_ctx);
    return true;
  }

  static bool CheckEndWithPunct(const std::string& s, const std::string& punct) {
    if (s.empty() || punct.empty()) return false;
    char last = s.back();
    return punct.find(last) != std::string::npos;
  }

  void PublishText(uint64_t req_id, const std::string& text) {
    std_msgs::msg::String msg;
    msg.data = "[sid:" + std::to_string(req_id) + "] " + text;
    text_pub_->publish(msg);
  }

  void PublishState(const std::string& state) {
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  // 成员变量
  std::string visual_hbm_;
  std::string audio_hbm_;
  std::string text_hbm_;
  std::string embed_tokens_;
  std::string tokenizer_dir_;

  int model_type_{5};
  bool online_mode_{true};
  int bpu_core_idx_{-1};
  int image_repeat_{kMinImageRepeat};
  double wait_timeout_s_{0.5};
  std::string punctuation_;
  std::string system_prompt_;

  std::string audio_topic_;
  std::string image_topic_;
  std::string text_topic_;
  std::string state_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr audio_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  std::mutex pair_mtx_;
  uint64_t global_req_id_{0};
  uint64_t waiting_req_id_{0};
  std::vector<uint8_t> waiting_wav_;
  rclcpp::Time wait_deadline_;
  bool waiting_image_{false};

  std::queue<InferRequest> infer_queue_;
  std::mutex req_mtx_;
  std::condition_variable req_cv_;
  std::thread worker_thread_;
  std::atomic<bool> stop_worker_{false};

  xlm_handle_t xlm_handle_{nullptr};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OmniMultimodalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}