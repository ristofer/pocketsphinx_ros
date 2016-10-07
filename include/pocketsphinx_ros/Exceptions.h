#ifndef EXCEPTIONS_HPP_
#define EXCEPTIONS_HPP_

// using standard exceptions
#include <iostream>
#include <exception>
using namespace std;

class open_device_error: public exception
{
  virtual const char* what() const throw()
  {
    return "Failed to open audio device";
  }
} ;

class start_recording_error: public exception
{
  virtual const char* what() const throw()
  {
    return "Failed to start recording audio";
  }
} ;

class read_audio_error: public exception
{
  virtual const char* what() const throw()
  {
    return "Failed to read audio";
  }
};

class init_config_error: public exception
{
  virtual const char* what() const throw()
  {
    return "Failed to create pocketsphinx config object"
  }
};

class init_ps_decoder_error: public exception
{
  virtual const char* what() const throw()
  {
    return "Failed to create pocketsphinx decoder object"
  }
};

class start_utt_error: public exception
{
  virtual const char* what() const throw()
  {
    return "Failed to start utterance in pocketsphinx recognizer"
  }
};


#endif /* EXCEPTIONS_HPP_ */