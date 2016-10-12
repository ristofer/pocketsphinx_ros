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


bool AudioSource::check_wav_header(char *header, int expected_sr)
{
    int sr;

    if (header[34] != 0x10) {
        E_ERROR("Input audio file has [%d] bits per sample instead of 16\n", header[34]);
        return false;
    }
    if (header[20] != 0x1) {
        E_ERROR("Input audio file has compression [%d] and not required PCM\n", header[20]);
        return false;
    }
    if (header[22] != 0x1) {
        E_ERROR("Input audio file has [%d] channels, expected single channel mono\n", header[22]);
        return false;
    }
    sr = ((header[24] & 0xFF) | ((header[25] & 0xFF) << 8) | ((header[26] & 0xFF) << 16) | ((header[27] & 0xFF) << 24));
    if (sr != expected_sr) {
        E_ERROR("Input audio file has sample rate [%d], but decoder expects [%d]\n", sr, expected_sr);
        return false;
    }
    return true;
}

void AudioSource::openFile(std::string fname)
{
    
    if ((rawfd_ = fopen(fname.c_str(), "rb")) == NULL) {
        E_FATAL_SYSTEM("Failed to open file '%s' for reading",
                       fname.c_str());
    }
}

void AudioSource::checkFile(std::string fname)
{
	   if (strlen(fname.c_str()) > 4 && strcmp(fname.c_str() + strlen(fname.c_str()) - 4, ".wav") == 0) 
    {
        char waveheader[44];
		fread(waveheader, 1, 44, rawfd_);
		if (!check_wav_header(waveheader, 16000))
		{
    	    E_FATAL("Failed to process file '%s' due to format mismatch.\n", fname.c_str());
		}
    }

    if (strlen(fname.c_str()) > 4 && strcmp(fname.c_str() + strlen(fname.c_str()) - 4, ".mp3") == 0) 
    {
	E_FATAL("Can not decode mp3 files, convert input file to WAV 16kHz 16-bit mono before decoding.\n");
    }
}

bool AudioSource::readFile()
{
	k_ = fread(buf_, sizeof(int16), 2048, rawfd_);
	return (k_>0);
    
}