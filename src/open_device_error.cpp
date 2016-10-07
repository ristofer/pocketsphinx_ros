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
} open_device_error;

int main () {
  try
  {
    throw open_device_error;
  }
  catch (exception& e)
  {
    cout << e.what() << '\n';
  }
  return 0;
}