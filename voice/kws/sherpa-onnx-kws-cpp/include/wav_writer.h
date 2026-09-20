#ifndef WAV_WRITER_H_
#define WAV_WRITER_H_

#include <vector>
#include <string>

// 保存 float32 采样为 16-bit PCM WAV 文件
bool SaveWav(const std::string& path, const std::vector<float>& samples, int sample_rate);

#endif  // WAV_WRITER_H_
