/*
 * recognizer.cpp
 *
 *  Created on: 01-09-2016
 *      Author: Cristopher Gómez
        Email: cristopher.gomez@ug.uchile.cl
 */

//ROS
#include <ros/ros.h>
#include <ros/package.h>

//Messages and services
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
//#include <bender_srvs/load_dictionary_service.h>
#include <pocketsphinx_ros/RecognitionOrder.h>

#include <sstream>


#include <stdio.h>
#include <string.h>
#include <assert.h>

//Sphinxbase for Pocketsphinx.  It's important to remember that Sphinxbase and Pocketsphinx are C libraries 
#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>

//Pocketsphinx
#include "pocketsphinx.h"



//AudioSource uses sphinxbase functions to acces the microphone and read the audio to give it to the recognizer object
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
        
        if ((ad_ = ad_open_dev(device_name.c_str(),16000)) == NULL) //Remember that ad_open_dev() it's a C function so it receive a c_str().
                                                                    //the second parameter 16000, it's the sampling rate
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

//Recognizer serves as a wrapper for the Pocketsphinx functions
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
            init_state_ = false; //this indicates if the pocketsphinx decoder (ps) is initialized
        }

    ~Recognizer()
    {
        cmd_ln_free_r(config_);
        ps_free(ps_);
        init_state_ = false;
        
    }

    void init()
    {
         config_ = cmd_ln_init(NULL, ps_args(), TRUE, // This structure saves the parameters for the decoder
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

        ps_ = ps_init(config_); //With the config structure the decoder is initialized
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

//RecognizerROS have services and publishers.  It wait for a service call to start the recognition and publish the results.
class RecognizerROS 
{
public:
    RecognizerROS(RecognizerPtr recognizer):
        recognizer_(recognizer),
        loop_rate_(100),
        nh_("~")
        
    {
        recognizer_->init();
        
        start_srv_ = nh_.advertiseService("start_with_dictionary",&RecognizerROS::startRecognition,this);
        stop_srv_ = nh_.advertiseService("stop",&RecognizerROS::stopRecognition,this);
        //load_dict_srv_ = nh_.advertiseService("load_dictionary",&RecognizerROS::loadDictionary,this);
        
        output_pub_ = nh_.advertise<std_msgs::String>("output",1,this);
        partial_output_pub = nh_.advertise<std_msgs::String>("partial_output",1,this);


        pkg_dir_= ros::package::getPath("pocketsphinx_ros");
        is_on_ = false;
    }
    ~RecognizerROS()
    {

    }

    bool startRecognition(pocketsphinx_ros::RecognitionOrder::Request &req, pocketsphinx_ros::RecognitionOrder::Response &res){
        loadDictionary(req.dictionary);
        recognize();
        
    }
   
    bool stopRecognition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        
    }

    void loadDictionary(std::string dict){

     updateDirectories(dict);
     recognizer_->setGrammar(grammardir_);
     recognizer_->setDict(dictdir_);
     recognizer_->update();
     
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
        std_msgs::String partial_result ;
        std_msgs::String final_result ;
        
        if (recognizer_->status() == false){return;}
        recognizer_->initDevice("alsa_input.usb-M-Audio_Producer_USB-00-USB.analog-stereo");

        recognizer_->startUtt();
        
        utt_started = FALSE;
        ROS_INFO_STREAM("Ready....");

        
        while(1){
            

            recognizer_->readAudio();
            recognizer_->proccesRaw();
            partial_result.data = recognizer_->getHyp();
            partial_output_pub.publish(partial_result);
           

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
                
                
                final_result.data = recognizer_->getHyp();

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
        output_pub_.publish(final_result);
        
     
    }

private:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    
    ros::ServiceServer start_srv_ ;
    ros::ServiceServer stop_srv_ ;
    ros::ServiceServer load_dict_srv_;

    ros::Publisher output_pub_;
    ros::Publisher partial_output_pub ;

    
    std::string pkg_dir_;



    std::string modeldir_;
    std::string grammardir_;
    std::string dictdir_;
    std::string threshold_;

    //std::string final_result_;
    //std::string partial_result_;
    

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
        "/home/robotica/bender_ws/soft_ws/src/bender_hri/pocketsphinx_ros/Grammar/Stage1/Stage2gpsr.jsgf",
        "/home/robotica/bender_ws/soft_ws/src/bender_hri/pocketsphinx_ros/Grammar/Stage1/Stage2gpsr.dic",
        "2.0"));
    
    RecognizerROS r(recognizer);


    
    
   
        
    
    ros::spin();
    
    
    // loop_rate.sleep();

    return 0;

}