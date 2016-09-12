#ifndef RECOGNIZERROS_HPP_
#define RECOGNIZERROS_HPP_


#include <ros/ros.h>

#include <boost/thread/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <pocketsphinx_ros/DoRecognitionAction.h>
#include <pocketsphinx_ros/SpeechRecognitionConfig.h>

#include <dynamic_reconfigure/server.h>

#include <sstream>

#include "pocketsphinx_ros/Recognizer.h"

typedef boost::shared_ptr<Recognizer> RecognizerPtr;

class RecognizerROS 
{
public:

    RecognizerROS();
    ~RecognizerROS();
    void executeCB(const pocketsphinx_ros::DoRecognitionGoalConstPtr &goal);
    void dynamicCallback(pocketsphinx_ros::SpeechRecognitionConfig &config,uint32_t level);
    void resetRecognizer();
    void updateDirectories(std::string dictionary);
    void recognize();

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