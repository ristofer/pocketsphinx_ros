#include <ros/ros.h>
#include <ros/package.h>

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
private:
    int16_t buf_[2048];
    ad_rec_t *ad_;
    int32 k_;

public:
    AudioSource(){}
    ~AudioSource(){}
    
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

    int16_t* buf()
    {
        return buf_;
    }

    ad_rec_t* ad()
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


};

class Recognizer
{
public:
    Recognizer(AudioSource *as, 
               std::string modeldir, 
               std::string grammardir, 
               std::string dictdir, 
               std::string threshold):
        as_(as),
        modeldir_(modeldir),
        grammardir_(grammardir),
        dictdir_(dictdir),
        threshold_(threshold)
        {
            init_state_ = false;
        }

    ~Recognizer()
    {
        cmd_ln_free_r(config_);
        ps_free(ps_);
        init_state_ = false;
        
    }

    void init()
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
        if (ps_ == NULL) 
        {
            fprintf(stderr, "Failed to create recognizer, see log for details\n");
        }

        init_state_ = true;
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
        ps_process_raw(ps_, as_->buf(), as_->k(), FALSE, FALSE);
    }

    void endUtt()
    {
        ps_end_utt(ps_);
    }

    
    std::string getHyp()
    {
         char const *hyp;
         hyp = ps_get_hyp(ps_, NULL);
         if(hyp==NULL){
            return std::string("");
         }
         return std::string(hyp);
    }

    bool inSpeech()
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
        as_->openDevice(device);
        as_->startRec();
    }

    void readAudio()
    {
        as_->read();
    }

    void terminateDevice()
    {
        as_->closeDevice();
    }

    void setDict(const std::string& dictdir)
    {
        dictdir_ = dictdir;
    }

    void setGrammar(const std::string& grammardir)
    {
        grammardir_ = grammardir;
    }

    void setThreshold(const std::string& threshold)
    {
        threshold_ = threshold;
    }

    void update()
    {
        if(init_state_){
        cmd_ln_free_r(config_);
        ps_free(ps_);
        init_state_ = false;
        }
        init();
    }

    bool status()
    {
        return init_state_;
    }


    
    
private:
    AudioSource *as_;
    int16_t buf_[2048];
    ps_decoder_t *ps_;
    cmd_ln_t *config_;
    
    std::string modeldir_;
    std::string grammardir_;
    std::string dictdir_;
    std::string threshold_;
    
    bool init_state_;



};


typedef boost::shared_ptr<Recognizer> RecognizerPtr;

class RecognizerROS 
{
public:
    RecognizerROS(RecognizerPtr recognizer):
        recognizer_(recognizer),
        loop_rate_(100)
    {
        recognizer_->init();
        
        start_srv_ = nh_.advertiseService("~start",&RecognizerROS::startRecognition,this);
        stop_srv_ = nh_.advertiseService("~stop",&RecognizerROS::stopRecognition,this);
       // load_dict_srv_ = nh_.advertiseService("~load_dictionary",&RecognizerROS::loadDictionary,this);
        
        output_pub_ = nh_.advertise<std_msgs::String>("~output",1,this);


        pkg_dir_= ros::package::getPath("bender_speech");
        is_on_ = false;
    }
    ~RecognizerROS()
    {

    }

    bool startRecognition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &
    res){
        
    }
   
    bool stopRecognition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        
    }

    // void executeCB(const pocketsphinx_ros::DoRecognitionGoalConstPtr &goal)
    // {
        
    //     std::string dictionary;
    //     dictionary = goal->dictionary;
    //     updateDirectories(dictionary);
    //     recognizer_->setGrammar(grammardir_);
    //     recognizer_->setDict(dictdir_);
    //     recognizer_->update();
    //     recognize();


    //}

    // void updateDirectories(std::string dictionary)
    // {
    //     std::stringstream ss;
    //     std::stringstream jsgf;
    //     std::stringstream dic;
    //     ss << pkg_dir_ << "/Grammar/" << dictionary ;
    //     std::string path = ss.str();
    //     jsgf << path << ".jsgf" ;
    //     dic << path << ".dic" ;
    //     dictdir_ = dic.str();
    //     grammardir_ = jsgf.str();
    // }


    void recognize()
    {
        uint8 utt_started;
        
        if (recognizer_->status() == false){return;}
        recognizer_->initDevice("alsa_input.usb-M-Audio_Producer_USB-00-USB.analog-stereo");

        recognizer_->startUtt();
        
        utt_started = FALSE;
        ROS_INFO_STREAM("Ready....");

        
        while(1){
            

            recognizer_->readAudio();
            recognizer_->proccesRaw();
            // feedback_.partial_result= recognizer_->getHyp();
            // action_server_.publishFeedback(feedback_);
           

            in_speech_ = recognizer_->inSpeech();
            
            if (in_speech_ && !utt_started) 
            {
                utt_started = TRUE;
                ROS_INFO_STREAM("Listening...");
            }
           
            

            if (!in_speech_ && utt_started)
            {
                
                recognizer_->endUtt();
                ROS_INFO_STREAM("Finishing");
                
                
                // result_.final_result = recognizer_->getHyp();

                break;
               /* if (final_result_ != NULL) 
                {
                    
                    is_on_ = false;
                    
                    break ;
                }*/
            }
            loop_rate_.sleep();
        }
        
        recognizer_->terminateDevice();
        // action_server_.setSucceeded(result_);
        
     
    }

private:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    
    ros::ServiceServer start_srv_ ;
    ros::ServiceServer stop_srv_ ;
    //ros::ServiceServer load_dict_srv_;

    ros::Publisher output_pub_;
    ros::Publisher partial_output_pub ;

    
    std::string pkg_dir_;



    std::string modeldir_;
    std::string grammardir_;
    std::string dictdir_;
    std::string threshold_;

    std::string final_result_;
    std::string partial_result_;
    

    RecognizerPtr recognizer_;
    bool in_speech_;
    bool is_on_;
    

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "recognizer");
    
    
    AudioSource as;
    

    RecognizerPtr recognizer( new Recognizer(&as,
        "/usr/local/share/pocketsphinx/model/en-us/en-us",
        "/home/bender/bender_ws/soft_ws/src/bender_hri/bender_speech/Grammar/Stage1/gpsr/Stage2gpsr.jsgf",
        "/home/bender/bender_ws/soft_ws/src/bender_hri/bender_speech/Grammar/Stage1/gpsr/Stage2gpsr.dic",
        "2.0"));
    
    RecognizerROS r(recognizer);


    
    
   
        
    
    ros::spin();
    
    
    // loop_rate.sleep();

    return 0;

}