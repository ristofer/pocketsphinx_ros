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
		ad_rect_t *ad;
		if ((ad = ad_open_dev(device_name.c_str(),16000)) == NULL)
		{
			E_FATAL("Failed to open audio device\n");
		}
		ad_ = ad;

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
		std::string modeldir = "/usr/local/share/pocketsphinx/model/en-us/en-us";
		config = cmd_ln_init(NULL, ps_args(), TRUE,
			"-hmm", modeldir.c_str(),
			"-jsgf","/home/superminion/CMU/Stage2gpsr.jsgf",
			"-dict","/home/superminion/CMU/0042.dic" ,
			"-vad_threshold","2.0",	
			"-remove_noise","yes",
			NULL);
		if (config == NULL)
		{
			fprintf(stderr, "Failed to create config object, see log for details\n");
			//return -1;
		}
	}

	~Recognizer()
	{
		cmd_ln_free_r(config);
	}

	std::string recognize()
	{
		
    for(;;){
	    if ((k = ad_read(ad, adbuf, 2048)) < 0)
	        E_FATAL("Failed to read audio\n");
	    
	    ps_process_raw(ps, adbuf, k, FALSE, FALSE);
	    partial = ps_get_hyp(ps, NULL);
	    ROS_WARN_STREAM(partial);

	    in_speech = ps_get_in_speech(ps);
	    
	    if (in_speech && !utt_started) {
	        utt_started = TRUE;
	        E_INFO("Listening...\n");
	    }
	   
	    

	 	if (!in_speech && utt_started) {
	        /* speech -> silence transition, time to start new utterance  */
	        ps_end_utt(ps);
	        hyp = ps_get_hyp(ps, NULL );
	        if (hyp != NULL) {
	            
	            return std::string(hyp);
	            
	        }

	        if (ps_start_utt(ps) < 0)
	            E_FATAL("Failed to start utterance\n");
	        utt_started = FALSE;
	        E_INFO("Ready....\n");
	    }
	    sleep_msec(100);
    }
    ad_close(ad);
 

    return std::string("");
	}
	
private:
	AudioSource as_;
	int16_t buf_[2048];
	ps_decoder_t *ps_;
	cmd_ln_t *config_;
};

