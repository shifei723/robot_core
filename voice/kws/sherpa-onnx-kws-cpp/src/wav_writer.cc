#include "wav_writer.h"
#include <fstream>
#include <cstdint>
#include <cstring>

static void WriteLE16(std::ofstream& f, uint16_t v) {
    char buf[2] = {static_cast<char>(v & 0xFF), static_cast<char>((v >> 8) & 0xFF)};
    f.write(buf, 2);
}

static void WriteLE32(std::ofstream& f, uint32_t v) {
    char buf[4];
    for (int i = 0; i < 4; ++i) buf[i] = static_cast<char>((v >> (8 * i)) & 0xFF);
    f.write(buf, 4);
}

bool SaveWav(const std::string& path, const std::vector<float>& samples, int sample_rate) {
    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) return false;

    int channels = 1;
    int bits_per_sample = 16;
    int num_samples = static_cast<int>(samples.size());
    int byte_rate = sample_rate * channels * bits_per_sample / 8;
    int block_align = channels * bits_per_sample / 8;
    int data_size = num_samples * block_align;
    int file_size = 36 + data_size;

    // RIFF header
    f.write("RIFF", 4);
    WriteLE32(f, file_size);
    f.write("WAVE", 4);

    // fmt chunk
    f.write("fmt ", 4);
    WriteLE32(f, 16);
    WriteLE16(f, 1);  // PCM
    WriteLE16(f, channels);
    WriteLE32(f, sample_rate);
    WriteLE32(f, byte_rate);
    WriteLE16(f, block_align);
    WriteLE16(f, bits_per_sample);

    // data chunk
    f.write("data", 4);
    WriteLE32(f, data_size);

    for (float s : samples) {
        int16_t v = static_cast<int16_t>(std::max(-1.0f, std::min(1.0f, s)) * 32767);
        WriteLE16(f, static_cast<uint16_t>(v));
    }

    return f.good();
}
