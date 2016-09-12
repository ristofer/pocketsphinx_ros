#ifndef RECOGNIZER_HPP_
#define RECOGNIZER_HPP_


#include <sphinxbase/err.h>
#include "pocketsphinx.h"
#include "pocketsphinx_ros/AudioSource.h"

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
               int vad_start);           
    ~Recognizer();
    void startUtt();
    void proccesRaw();
    void endUtt();
    std::string getHyp();
    bool inSpeech();
    void initDevice(std::string device);
    void readAudio();
    void terminateDevice();
    void setDict(const std::string& dictdir);
    void setGrammar(const std::string& grammardir);
    void setThreshold(const std::string& threshold);
    bool status();
    void update();
    std::string getSearch();
     
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


#endif /* RECOGNIZER_HPP_ */