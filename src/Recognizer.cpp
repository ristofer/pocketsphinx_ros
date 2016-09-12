#include "pocketsphinx_ros/Recognizer.h"
#include <boost/lexical_cast.hpp>

Recognizer::Recognizer(AudioSource *as, 
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
        prespeech_ = boost::lexical_cast<std::string>(vad_pre);
        postspeech_ = boost::lexical_cast<std::string>(vad_post);
        startspeech_ = boost::lexical_cast<std::string>(vad_start);
       
        //ROS_INFO_STREAM(threshold_.c_str());


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



Recognizer::~Recognizer()
{
    cmd_ln_free_r(config_);
    ps_free(ps_);
    init_state_ = false;
}



void Recognizer::startUtt()
{
    if (ps_start_utt(ps_) < 0)
    {
        E_FATAL("Failed to start utterance\n");
    }
}

void Recognizer::proccesRaw()
{
    ps_process_raw(ps_, as_->buf(), as_->k(), FALSE, FALSE);
}

void Recognizer::endUtt()
{
    ps_end_utt(ps_);
}


std::string Recognizer::getHyp()
{
     char const *hyp;
     hyp = ps_get_hyp(ps_, NULL);
     if(hyp==NULL){
        return std::string("");
     }
     return std::string(hyp);
}

bool Recognizer::inSpeech()
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

void Recognizer::initDevice(std::string device)
{
    as_->openDevice(device);
    as_->startRec();
}

void Recognizer::readAudio()
{
    as_->read();
}

void Recognizer::terminateDevice()
{
    as_->closeDevice();
}

void Recognizer::setDict(const std::string& dictdir)
{
    dictdir_ = dictdir;
    ps_load_dict(ps_,dictdir_.c_str(),NULL,NULL);
}

void Recognizer::setGrammar(const std::string& grammardir)
{
    grammardir_ = grammardir;
    ps_set_jsgf_file(ps_,"grammar_search",grammardir_.c_str());    
    ps_set_search(ps_,"grammar_search");
}


void Recognizer::setThreshold(const std::string& threshold)
{
    threshold_ = threshold;
}


bool Recognizer::status()
{
    return init_state_;
}

void Recognizer::update()
{
    if(init_state_){
    cmd_ln_free_r(config_);
    ps_free(ps_);
    init_state_ = false;
    }
    
}



std::string Recognizer::getSearch()
{
    

     char const *search_name;
     search_name = ps_get_search(ps_);
     if(search_name==NULL){
        return std::string("");
     }
     return std::string(search_name);

}
