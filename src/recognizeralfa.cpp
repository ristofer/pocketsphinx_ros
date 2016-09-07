#include <ros/ros.h>

#include <boost/thread/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <pocketsphinx_ros/DoRecognitionAction.h>
#include <pocketsphinx_ros/SpeechRecognitionConfig.h>

#include <dynamic_reconfigure/server.h>

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
               double vad_thres,
               int vad_pre,
               int vad_post,
               int vad_start):
        as_(as),
        modeldir_(modeldir),
        grammardir_(grammardir),
        dictdir_(dictdir)
        {
            threshold_ = boost::lexical_cast<std::string>(vad_thres);
            prespeech_ = boost::to_string(vad_pre);
            postspeech_ = boost::to_string(vad_post);
            startspeech_ = boost::to_string(vad_start);
           
            ROS_INFO_STREAM(threshold_.c_str());


            config_ = cmd_ln_init(NULL, ps_args(), TRUE,
            "-hmm", modeldir_.c_str(),
            "-jsgf",grammardir_.c_str(),
            "-dict",dictdir_.c_str() ,
            "-vad_threshold",threshold_.c_str(),
            "-vad_prespeech",prespeech_.c_str(),
            "-vad_postspeech",postspeech_.c_str(),
            "-vad_startspeech",startspeech_.c_str(),    
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



    ~Recognizer()
    {
        cmd_ln_free_r(config_);
        ps_free(ps_);
        init_state_ = false;
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
        ps_load_dict(ps_,dictdir_.c_str(),NULL,NULL);
    }

    void setGrammar(const std::string& grammardir)
    {
        grammardir_ = grammardir;
        ps_set_jsgf_file(ps_,"grammar_search",grammardir_.c_str());    
        ps_set_search(ps_,"grammar_search");
    }

    



    void setThreshold(const std::string& threshold)
    {
        threshold_ = threshold;
    }


    bool status()
    {
        return init_state_;
    }

    void update()
    {
        if(init_state_){
        cmd_ln_free_r(config_);
        ps_free(ps_);
        init_state_ = false;
        }
        //init();
    }

    // void init(double vad_thres, int vad_pre, int vad_post, int vad_start)
    // {
            
    //     threshold_ = boost::lexical_cast<std::string>(vad_thres);
    //     prespeech_ = boost::to_string(vad_pre);
    //     postspeech_ = boost::to_string(vad_post);
    //     startspeech_ = boost::to_string(vad_start);
       
    //     ROS_INFO_STREAM(threshold_.c_str());


    //     config_ = cmd_ln_init(NULL, ps_args(), TRUE,
    //     "-hmm", modeldir_.c_str(),
    //     "-jsgf",grammardir_.c_str(),
    //     "-dict",dictdir_.c_str() ,
    //     "-vad_threshold",threshold_.c_str(),
    //     "-vad_prespeech",prespeech_.c_str(),
    //     "-vad_postspeech",postspeech_.c_str(),
    //     "-vad_startspeech",startspeech_.c_str(),    
    //     "-remove_noise","yes",
    //     NULL);
        
    //     if (config_ == NULL)
    //     {
    //         fprintf(stderr, "Failed to create config object, see log for details\n");
    //         //return -1;
    //     }

    //     ps_ = ps_init(config_);
    //     if (ps_ == NULL) 
    //     {
    //         fprintf(stderr, "Failed to create recognizer, see log for details\n");
    //     }

    //     init_state_ = true;

    // }

    std::string getSearch()
    {
        

         char const *search_name;
         search_name = ps_get_search(ps_);
         if(search_name==NULL){
            return std::string("");
         }
         return std::string(search_name);
    
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
    std::string prespeech_;
    std::string postspeech_;
    std::string startspeech_;

    
    bool init_state_;




};


typedef boost::shared_ptr<Recognizer> RecognizerPtr;

class RecognizerROS 
{
public:

    RecognizerROS():
        nh_("~"),
        actionServer_(nh_, "recognizer", boost::bind(&RecognizerROS::executeCB, this, _1), false),
        loop_rate_(100)
    {
        is_on_ = false;

        
        if(nh_.hasParam("hmmdir"))
        {
            nh_.getParam("hmmdir",hmmdir_);
        }
        if(nh_.hasParam("packagedir"))
        {
        nh_.getParam("packagedir",pkg_dir_);
        }
       
        

        updateDirectories("Stage1/Stage2gpsr");

 

        
        actionServer_.start();
        reconfigureCallback_ = boost::bind(&RecognizerROS::dynamicCallback,this, _1, _2);
        parameterServer_.setCallback(reconfigureCallback_);
        
        

         
       
        
        
    }


    ~RecognizerROS()
    {

    }


    void executeCB(const pocketsphinx_ros::DoRecognitionGoalConstPtr &goal)
    {
        
        std::string dictionary_name;
        dictionary_name = goal->dictionary;
        updateDirectories(dictionary_name);
        recognizer_->setGrammar(grammardir_);
        recognizer_->setDict(dictdir_);
        //recognizer_->update();
        recognize();

    }
    void dynamicCallback(pocketsphinx_ros::SpeechRecognitionConfig &config,uint32_t level)
    {
        
        
        // pkg_dir_= ros::package::getPath("pocketsphinx_ros");

        // updateDirectories("Stage1/Stage2gpsr");
        while(is_on_){};
        vad_thres_ = config.vad_threshold;
        vad_pre_ = config.vad_prespeech;
        vad_post_ = config.vad_postspeech;
        vad_start_ = config.vad_startspeech;
        //boost::thread thread_b(&RecognizerROS::resetRecognizer,this);
        resetRecognizer();
        // recognizer_.reset(new Recognizer(&as_,
        // "/usr/local/share/pocketsphinx/model/en-us/en-us",
        // grammardir_,
        // dictdir_,
        // vad_thres_,
        // vad_pre_,
        // vad_post_,
        // vad_start_));
    }

    void resetRecognizer()
    {
  

        recognizer_.reset(new Recognizer(&as_,
        hmmdir_,
        grammardir_,
        dictdir_,
        vad_thres_,
        vad_pre_,
        vad_post_,
        vad_start_));
    }

    

    void updateDirectories(std::string dictionary)
    {
        std::stringstream ss;
        std::stringstream jsgf;
        std::stringstream dic;
        ss << pkg_dir_ << "/Grammar/" << dictionary ;
        std::string path = ss.str();
        jsgf << path << ".jsgf" ;
        dic << path << ".dic" ;
        dictdir_ = dic.str();
        grammardir_ = jsgf.str();
    }



    void recognize()
    {
        uint8 utt_started;
        std::string search_name;

        is_on_ = true;
        if (recognizer_->status() == false){return;}

        recognizer_->initDevice("alsa_input.usb-M-Audio_Producer_USB-00-USB.analog-stereo");

        recognizer_->startUtt();
        
        utt_started = FALSE;

        search_name = recognizer_->getSearch() ;

        ROS_INFO_STREAM(search_name);

        ROS_INFO_STREAM("Ready....");


        
        while(1){
            

            recognizer_->readAudio();
            recognizer_->proccesRaw();

            feedback_.partial_result = recognizer_->getHyp();
            actionServer_.publishFeedback(feedback_);

           

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
                
                
                result_.final_result = recognizer_->getHyp();


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

        actionServer_.setSucceeded(result_);
        is_on_=false;
     
    }

private:

    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    AudioSource as_;

    actionlib::SimpleActionServer<pocketsphinx_ros::DoRecognitionAction> actionServer_;

    pocketsphinx_ros::DoRecognitionFeedback feedback_;
    pocketsphinx_ros::DoRecognitionResult result_;

    
    dynamic_reconfigure::Server<pocketsphinx_ros::SpeechRecognitionConfig> parameterServer_;
    dynamic_reconfigure::Server<pocketsphinx_ros::SpeechRecognitionConfig>::CallbackType reconfigureCallback_;





    std::string pkg_dir_;


    std::string hmmdir_;
    std::string modeldir_;
    std::string grammardir_;
    std::string dictdir_;
    std::string threshold_;

    double vad_thres_;
    int vad_pre_;
    int vad_post_;
    int vad_start_;
    


    std::string final_result_;
    std::string partial_result_;
    


    RecognizerPtr recognizer_;
    bool in_speech_;
    bool is_on_;
    

};


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "recognizer");
    
  
    


    RecognizerROS r;



    
    
    ros::spin();

    
    
    // loop_rate.sleep();

    return 0;

}