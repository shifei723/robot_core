#ifndef AUDIO_CAPTURE_H_
#define AUDIO_CAPTURE_H_

#include <vector>

// PortAudio 音频采集封装
class AudioCapture {
public:
    AudioCapture(int sample_rate, int chunk_size, int channels = 1);
    ~AudioCapture();

    // 读取一个 chunk，返回 float32 采样 [-1, 1]
    std::vector<float> Read();

    // 是否打开成功
    bool IsOpen() const;

private:
    struct Impl;
    Impl* impl_;
};

#endif  // AUDIO_CAPTURE_H_
