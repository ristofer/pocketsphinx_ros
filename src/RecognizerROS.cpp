#include <pocketsphinx_ros/RecognizerROS.hpp>


RecognizerROS::RecognizerROS():
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
   
    if(nh_.hasParam("mic_name"))
    {
        nh_.getParam("mic_name",mic_name_);
    }

    updateDirectories("Stage1/Stage2gpsr");



    
    actionServer_.start();
    reconfigureCallback_ = boost::bind(&RecognizerROS::dynamicCallback,this, _1, _2);
    parameterServer_.setCallback(reconfigureCallback_);
        
}


RecognizerROS::~RecognizerROS()
{

}


void RecognizerROS::executeCB(const pocketsphinx_ros::DoRecognitionGoalConstPtr &goal)
{
    
    std::string dictionary_name;
    dictionary_name = goal->dictionary;
    updateDirectories(dictionary_name);
    recognizer_->setGrammar(grammardir_);
    recognizer_->setDict(dictdir_);
    //recognizer_->update();
    recognize();

}
void RecognizerROS::dynamicCallback(pocketsphinx_ros::SpeechRecognitionConfig &config,uint32_t level)
{
    
    
    // pkg_dir_= ros::package::getPath("pocketsphinx_ros");

    // updateDirectories("Stage1/Stage2gpsr");
    while(is_on_ && ros::ok()){};
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



void RecognizerROS::recognize()
{
    uint8 utt_started;
    std::string search_name;

    is_on_ = true;
    if (recognizer_->status() == false){return;}

    recognizer_->initDevice(mic_name_);

    recognizer_->startUtt();
    
    utt_started = FALSE;

    search_name = recognizer_->getSearch() ;

    ROS_INFO_STREAM(search_name);

    ROS_INFO_STREAM("Ready....");


    
    while(ros::ok()){
        

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

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "recognizer");
    
  
    


    RecognizerROS r;



    
    
    ros::spin();

    
    
    // loop_rate.sleep();

    return 0;

}