#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <sstream>


#include <stdio.h>
#include <string.h>
#include <assert.h>


#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>

#include "pocketsphinx.h"


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
		return buf_;
	}

	ad_rect_t ad()
	{
		return ad_;
	}

	int32 k()
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
	Recognizer(const AudioSource& as, 
			   std::string modeldir, 
			   std::string grammardir, 
			   std::string dictdir, 
			   std::string threshold):
		as_(as),
		modeldir_(modeldir),
		grammardir_(grammardir)
		dictdir_(dictdir),
		threshold_(threshold)

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

	void startUtt()
	{
		if (ps_start_utt(ps_) < 0)
	    {
        	E_FATAL("Failed to start utterance\n");
   		}
	}

	void proccesRaw()
	{
		ps_process_raw(ps_, as_.buf(), as_.k(), FALSE, FALSE);
	}

	void endUtt()
	{
		ps_end_utt(ps_);
	}

	std::string getHyp()
	{
		 char const *hyp;
		 hyp = ps_get_hyp(ps_, NULL);
		 return std::string(hyp);
	}

	bool inSpeech():
	{
		if (ps_get_in_speech(ps_))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void initDevice(std::string device)
	{
		as_.openDevice(device);
		as_.startRec();
	}

	void readAudio()
	{
		as_.read();
	}

	void terminateDevice()
	{
		as_.closeDevice();
	}


	
	
private:
	AudioSource as_;
	int16_t buf_[2048];
	ps_decoder_t *ps_;
	cmd_ln_t *config_;
	std::string modeldir_;
	std::string grammardir_;
	std::string dictdir_;
	std::string threshold_;
	std::string partial_;



};

class RecognizerROS 
{
public:
	RecognizerROS(const Recognizer& recognizer):
		recognizer_(recognizer)
	{
		is_on_ = false ;

		output_pub_ = n_.advertise<std_msgs::String>("~output",1000);
		partial_output_pub_ = n_.advertise<std_msgs::String>("~partial_output",1000);
		start_service_ = n_.advertiseService("~start", &RecognizerROS::start, this);
		stop_service_ = n_.advertiseService("~stop", &RecognizerROS::stop, this );
		load_dictionary_service_ = n_.advertiseService("~load_dictionary", &);



	}
	~RecognizerROS()
	{

	}

	void start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
	{
		is_on_ = true;
		recognize() ;
	}

	void stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
	{
		is_on_ = false ;
	}

	void recognize()
	{
		uint8 utt_started;
		
		
		recognizer_.initDevice("alsa_input.usb-M-Audio_Producer_USB-00-USB.analog-mono");

		recognizer_.startUtt();
   		
   		
   		utt_started = FALSE;
		E_INFO("Ready....\n");

    
    while(is_on_){
	    

	    recognizer_.readAudio();
	    recognizer_.proccesRaw();
	    partial_result_.data = recognizer_.getHyp();
	    partial_output_pub_.publish(partial_result_);
	   

	    in_speech_ = recognizer_.inSpeech();
	    
	    if (in_speech_ && !utt_started) 
	    {
	        utt_started = TRUE;
	        E_INFO("Listening...\n");
	    }
	   
	    

	 	if (!in_speech_ && utt_started)
	 	{
	        
	        recognizer_.endUtt();
	        
	        final_result_.data= recognizer_.getHyp();
	        
	        if (final_result_.data.c_str() != NULL) 
	        {
	        	output_pub_.publish(final_result_);
	        	is_on_ = false;
	        	break ;
	        }
	    }
	    recognition_loop_rate_.sleep();
    }
    recognizer_.terminateDevice();
 
	}

private:
	ros::NodeHandle n_;
	ros::Publisher output_pub_, partial_output_pub_;
	ros::ServiceServer start_service_,stop_service_,load_dictionary_service_;
	ros::Rate recognition_loop_rate_(100);
	std_msgs::String final_result_;
	std_msgs::String partial_result_;
	Recognizer recognizer_;
	bool in_speech_;
	bool is_on_
	

};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "talker");
	ros::Rate loop_rate(10);

	RecognizerROS r;
	
	ros::spin();
	loop_rate.sleep();

	return 0;

}