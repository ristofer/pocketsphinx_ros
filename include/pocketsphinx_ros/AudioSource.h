#ifndef AUDIOSOURCE_HPP_
#define AUDIOSOURCE_HPP_

#include <sphinxbase/ad.h>
#include <sphinxbase/err.h>

#include <string>

class AudioSource
{
private:
    int16_t buf_[2048];
    ad_rec_t *ad_;
    int32 k_;

public:
    AudioSource();
    ~AudioSource();  
    void openDevice(std::string device_name);
    void startRec();
    void read();
    int16_t* buf();
    ad_rec_t* ad();
    int32 k();
	void closeDevice();
};

#endif /* AUDIOSOURCE_HPP_ */