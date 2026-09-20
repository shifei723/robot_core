#include "audio_capture.h"
#include <portaudio.h>
#include <cstring>
#include <stdexcept>

struct AudioCapture::Impl {
    PaStream* stream = nullptr;
    int sample_rate;
    int chunk_size;
    int channels;
    std::vector<float> buffer;
};

AudioCapture::AudioCapture(int sample_rate, int chunk_size, int channels)
    : impl_(new Impl{nullptr, sample_rate, chunk_size, channels}) {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        throw std::runtime_error(std::string("PortAudio init failed: ") + Pa_GetErrorText(err));
    }

    PaStreamParameters params;
    std::memset(&params, 0, sizeof(params));
    params.device = Pa_GetDefaultInputDevice();
    if (params.device == paNoDevice) {
        throw std::runtime_error("No default input device found");
    }
    params.channelCount = channels;
    params.sampleFormat = paInt16;
    params.suggestedLatency = Pa_GetDeviceInfo(params.device)->defaultLowInputLatency;
    params.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&impl_->stream, &params, nullptr,
                        sample_rate, chunk_size, paClipOff, nullptr, nullptr);
    if (err != paNoError) {
        throw std::runtime_error(std::string("PortAudio open stream failed: ") + Pa_GetErrorText(err));
    }

    err = Pa_StartStream(impl_->stream);
    if (err != paNoError) {
        throw std::runtime_error(std::string("PortAudio start stream failed: ") + Pa_GetErrorText(err));
    }
}

AudioCapture::~AudioCapture() {
    if (impl_) {
        if (impl_->stream) {
            Pa_StopStream(impl_->stream);
            Pa_CloseStream(impl_->stream);
        }
        Pa_Terminate();
        delete impl_;
    }
}

std::vector<float> AudioCapture::Read() {
    std::vector<int16_t> raw(impl_->chunk_size * impl_->channels);
    PaError err = Pa_ReadStream(impl_->stream, raw.data(), impl_->chunk_size);
    if (err != paNoError && err != paInputOverflowed) {
        throw std::runtime_error(std::string("PortAudio read failed: ") + Pa_GetErrorText(err));
    }

    impl_->buffer.resize(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
        impl_->buffer[i] = static_cast<float>(raw[i]) / 32768.0f;
    }
    return impl_->buffer;
}

bool AudioCapture::IsOpen() const {
    return impl_ && impl_->stream != nullptr;
}
