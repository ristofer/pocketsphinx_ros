#include "uchile_speech_pocketsphinx/RecognizerROS.h"


RecognizerROS::RecognizerROS():
    nh_("~"),
    actionServer_(nh_, "recognizer_action", boost::bind(&RecognizerROS::executeCB, this, _1), false),
    loop_rate_(100)
{
    is_on_ = false;

    uchile_util::ParameterServerWrapper psw;
    psw.getParameter("hmmdir", hmmdir_, hmmdir_);
    psw.getParameter("mic_name", mic_name_, mic_name_);
    pkg_dir_ = ros::package::getPath("uchile_speech_pocketsphinx");
  
    // default dictionary
    updateDirectories("Stage1/GPSR/Stage2gpsr");

    actionServer_.start();
    reconfigureCallback_ = boost::bind(&RecognizerROS::dynamicCallback,this, _1, _2);
    parameterServer_.setCallback(reconfigureCallback_);
}


RecognizerROS::~RecognizerROS() { }


void RecognizerROS::executeCB(const uchile_speech_pocketsphinx::DoRecognitionGoalConstPtr &goal)
{
    std::string dictionary_name;
    dictionary_name = goal->dictionary;
    updateDirectories(dictionary_name);
    //recognizer_->setDict(dictdir_);
    //recognizer_->setGrammar(grammardir_);
    resetRecognizer();
    //recognizer_->update();
    if(goal->file != "")
    {
        recognizeFile(10,goal->file);
    }
    else
    {
        recognize();
    }    
}
void RecognizerROS::dynamicCallback(uchile_speech_pocketsphinx::SpeechRecognitionConfig &config,uint32_t level)
{   
    // pkg_dir_= ros::package::getPath("uchile_speech_pocketsphinx");

    // updateDirectories("Stage1/Stage2gpsr");
    
    vad_thres_ = config.vad_threshold;
    vad_pre_ = config.vad_prespeech;
    vad_post_ = config.vad_postspeech;
    vad_start_ = config.vad_startspeech;
    //boost::thread thread_b(&RecognizerROS::resetRecognizer,this);
    if(!is_on_)
    {
        resetRecognizer();
    }
    // recognizer_.reset(new Recognizer(&as_,
    // "/usr/local/share/pocketsphinx/model/en-us/en-us",
    // grammardir_,
    // dictdir_,
    // vad_thres_,
    // vad_pre_,
    // vad_post_,
    // vad_start_));
}

void RecognizerROS::resetRecognizer()
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



void RecognizerROS::updateDirectories(std::string dictionary)
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



void RecognizerROS::recognize(double timeout)
{
    uint8 utt_started;
    std::string search_name;
    ros::Time begin = ros::Time::now();

    is_on_ = true;
    if (recognizer_->status() == false) { return; }

    try {
        recognizer_->initDevice(mic_name_);
    }
    catch(exception& e) {
        is_on_ = false;
        ROS_ERROR_STREAM(e.what());
        result_.final_result = "";
        actionServer_.setAborted(result_);
        return;
    }

    recognizer_->startUtt();
    utt_started = FALSE;
    search_name = recognizer_->getSearch() ;

    ROS_INFO_STREAM(search_name);
    ROS_INFO_STREAM("Ready....");
    
    while(ros::ok())
    {
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
            ROS_INFO_STREAM(result_.final_result);
            break;
           /* if (final_result_ != NULL) 
            {
                
                is_on_ = false;
                
                break ;
            }*/
        }

        if ((ros::Time::now()-begin).toSec() > timeout)
        {
            if (utt_started){recognizer_->endUtt();}
            recognizer_->terminateDevice();
            ROS_INFO_STREAM("Timeout");
            result_.final_result = "";
            actionServer_.setAborted();
            is_on_=false;
            return;
        }
        
        loop_rate_.sleep();
    }
    
    recognizer_->terminateDevice();

    actionServer_.setSucceeded(result_);
    is_on_=false;
 
}

void RecognizerROS::recognizeFile(double timeout, std::string fname)
{
    ROS_INFO_STREAM("Recognize file");
    uint8 utt_started;
    std::string search_name;
    ros::Time begin = ros::Time::now();

    is_on_ = true;
    if (recognizer_->status() == false) { return; }

    try {
        recognizer_->initFile(fname);
    }
    catch(exception& e) {
        is_on_ = false;
        ROS_ERROR_STREAM(e.what());
        result_.final_result = "";
        actionServer_.setAborted(result_);
        return;
    }

    recognizer_->startUtt();
    utt_started = FALSE;
    search_name = recognizer_->getSearch() ;

    ROS_INFO_STREAM(search_name);
    ROS_INFO_STREAM("Ready....");
    
    while(ros::ok() and recognizer_->readAudioFromFile())
    {
    
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
            ROS_INFO_STREAM(result_.final_result);
            break;
           /* if (final_result_ != NULL) 
            {
                
                is_on_ = false;
                
                break ;
            }*/
        }

        if ((ros::Time::now()-begin).toSec() > timeout)
        {
            if (utt_started){recognizer_->endUtt();}
            recognizer_->terminateDevice();
            ROS_INFO_STREAM("Timeout");
            result_.final_result = "";
            actionServer_.setAborted();
            is_on_=false;
            return;
        }
        
        loop_rate_.sleep();
    }
    
    
    recognizer_->endUtt();
    if (utt_started) 
    {
        result_.final_result = recognizer_->getHyp();
        ROS_INFO_STREAM("Finishing");
        ROS_INFO_STREAM(result_.final_result);
    }
    actionServer_.setSucceeded(result_);
    is_on_=false;
    recognizer_->terminateFile();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "recognizer");
    RecognizerROS r; 
    ros::spin();
    // loop_rate.sleep();
    return 0;
}