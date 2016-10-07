#include "pocketsphinx_ros/AudioSource.h"


AudioSource::AudioSource(){}
AudioSource::~AudioSource(){}

void AudioSource::openDevice(std::string device_name)
{
    
    if ((ad_ = ad_open_dev(device_name.c_str(),16000)) == NULL)
    {
        throw open_device_error();
    }
}

void AudioSource::startRec()
{
    if (ad_start_rec(ad_) < 0)
    {
        throw start_recording_error();
    }
}

void AudioSource::read()
{
    if ((k_ = ad_read(ad_, buf_, 2048)) < 0)
    {
        throw read_audio_error();
    }

}

int16_t* AudioSource::buf()
{
    return buf_;
}

ad_rec_t* AudioSource::ad()
{
    return ad_;
}

int32 AudioSource::k()
{
    return k_;
}

void AudioSource::closeDevice()
{
    ad_close(ad_);
}
