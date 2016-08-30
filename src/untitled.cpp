#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>


#include <stdio.h>
#include <string.h>
#include <assert.h>


#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>

#include "pocketsphinx.h"

static ps_decoder_t *ps;
static cmd_ln_t *config;

static void
sleep_msec(int32 ms)
{
#if (defined(_WIN32) && !defined(GNUWINCE)) || defined(_WIN32_WCE)
    Sleep(ms);
#else
    /* ------------------- Unix ------------------ */
    struct timeval tmo;

    tmo.tv_sec = 0;
    tmo.tv_usec = ms * 1000;

    select(0, NULL, NULL, NULL, &tmo);
#endif
}

class AudioSource
{
public:
	AudioSource();
	~AudioSource();

	void openDevice(std::string device_name)
	{
		
		if ((ad_ = ad_open_dev(device_name.c_str(),16000)) == NULL)
		{
			E_FATAL("Failed to open audio device\n");
		}
		

	}

	void startRec()
	{
		if (ad_start_rec(ad_) < 0)
		{
        	E_FATAL("Failed to start recording\n");
        }
	}


	void read()
	{
		if ((k_ = ad_read(ad_, buf_, 2048)) < 0)
		{
	        E_FATAL("Failed to read audio\n");
		}
	}

	int16_t buf()
	{
		return buf_[2048];
	}

	ad_rect_t ad()
	{
		return ad_;
	}

	int32 k();
	{
		return k_;
	}

	void closeDevice()
	{
		ad_close(ad_);
	}

private:
	int16_t buf_[2048];
	ad_rec_t *ad_;
	int32 k_;
};

class Recognizer
{
public:
	Recognizer(const AudioSource& as):
		as_(as)
	{
		
		config_ = cmd_ln_init(NULL, ps_args(), TRUE,
			"-hmm", modeldir_.c_str(),
			"-jsgf",grammardir_.c_str(),
			"-dict",dictdir_.c_str() ,
			"-vad_threshold",threshold_.c_str(),	
			"-remove_noise","yes",
			NULL);
		if (config_ == NULL)
		{
			fprintf(stderr, "Failed to create config object, see log for details\n");
			//return -1;
		}

		ps_ = ps_init(config_);
		if (ps == NULL) 
		{
        	fprintf(stderr, "Failed to create recognizer, see log for details\n");
        }
	}

	~Recognizer()
	{
		cmd_ln_free_r(config_);
		ps_free(ps_);
	}

	std::string recognize()
	{
		uint8 utt_started, in_speech;
		char const *hyp;
		
		as_.openDevice("alsa_input.usb-M-Audio_Producer_USB-00-USB.analog-mono");
	    as_.startRec()

	    if (ps_start_utt(ps_) < 0)
	    {
        	E_FATAL("Failed to start utterance\n");
   		}
   		utt_started = FALSE;
		E_INFO("Ready....\n");

    
    for(;;){
	    

	    as_.read()
	    ps_process_raw(ps_, as_.buf(), as_.k(), FALSE, FALSE);
	    partial_ = ps_get_hyp(ps_, NULL);
	   

	    in_speech = ps_get_in_speech(ps_);
	    
	    if (in_speech && !utt_started) {
	        utt_started = TRUE;
	        E_INFO("Listening...\n");
	    }
	   
	    

	 	if (!in_speech && utt_started) {
	        /* speech -> silence transition, time to start new utterance  */
	        ps_end_utt(ps_);
	        
	        hyp = ps_get_hyp(ps_, NULL );
	        
	        if (hyp != NULL) 
	        {
	        	return std::string(hyp);
	        }

	        if (ps_start_utt(ps_) < 0)
	            E_FATAL("Failed to start utterance\n");
	        utt_started = FALSE;
	        E_INFO("Ready....\n");
	    }
	    sleep_msec(100);
    }
    as_.closeDevice()
 

    return std::string("");
	}
	
private:
	AudioSource as_;
	int16_t buf_[2048];
	ps_decoder_t *ps_;
	cmd_ln_t *config_;
	std::string modeldir_
	std::string grammardir_
	std::string dictdir_
	std::string threshold_
	std::string partial_


};

