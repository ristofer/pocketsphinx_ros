#ifndef AUDIOSOURCE_HPP_
#define AUDIOSOURCE_HPP_

#include <sphinxbase/ad.h>
#include <sphinxbase/err.h>
#include "pocketsphinx_ros/Exceptions.h"

#include <string>
#include <string.h>

class AudioSource
{
private:
    int16_t buf_[2048];
    ad_rec_t *ad_;
    int32 k_;
    FILE *rawfd_;

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
    bool check_wav_header(char *header, int expected_sr);
    void openFile(std::string fname);
    void checkFile(std::string fname);
    bool readFile();
};

#endif /* AUDIOSOURCE_HPP_ */