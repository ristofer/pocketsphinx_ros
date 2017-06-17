#ifndef RECOGNIZERROS_HPP_
#define RECOGNIZERROS_HPP_


#include <ros/ros.h>
#include <ros/package.h>

#include <boost/thread/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <uchile_speech_pocketsphinx/DoRecognitionAction.h>
#include <uchile_speech_pocketsphinx/SpeechRecognitionConfig.h>

#include <dynamic_reconfigure/server.h>

#include <sstream>

#include "uchile_speech_pocketsphinx/Recognizer.h"
#include <uchile_util/ParameterServerWrapper.h>

typedef boost::shared_ptr<Recognizer> RecognizerPtr;

class RecognizerROS 
{
public:

    RecognizerROS();
    ~RecognizerROS();
    void executeCB(const uchile_speech_pocketsphinx::DoRecognitionGoalConstPtr &goal);
    void dynamicCallback(uchile_speech_pocketsphinx::SpeechRecognitionConfig &config,uint32_t level);
    void resetRecognizer();
    void updateDirectories(std::string dictionary);
    void recognize(double timeout=15.0);
    void recognizeFile(double timeout, std::string fname);

private:

    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    AudioSource as_;

    actionlib::SimpleActionServer<uchile_speech_pocketsphinx::DoRecognitionAction> actionServer_;

    uchile_speech_pocketsphinx::DoRecognitionFeedback feedback_;
    uchile_speech_pocketsphinx::DoRecognitionResult result_;

    dynamic_reconfigure::Server<uchile_speech_pocketsphinx::SpeechRecognitionConfig> parameterServer_;
    dynamic_reconfigure::Server<uchile_speech_pocketsphinx::SpeechRecognitionConfig>::CallbackType reconfigureCallback_;


    std::string pkg_dir_;

    std::string hmmdir_;
    std::string modeldir_;
    std::string grammardir_;
    std::string dictdir_;
    std::string threshold_;
    std::string mic_name_;

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

#endif /* RECOGNIZERROS_HPP_ */