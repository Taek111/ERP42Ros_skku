#include <iostream>
#include <sstream>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <vector>
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions (struct termios)
#include <system_error>	// For throwing std::system_error

// TODO:
// too much including on header. this will increase compile time and binary size later... 
// move privatly using headers to inside the 'SerialPort.cpp' file
// I think vector and string is enough for now.

using namespace std;

class SerialPort
{
public:
  SerialPort(const char *device_name);
  virtual ~SerialPort(void);

  void Open(const char *device_name);
  void Close();
  void Configure();
  
  
  void Read(string& data);
  void Write(const string& data);
  void ReadPacket(string& data);
  
private:
  int fd; // File Descriptor
  vector<char> readBuffer;
  const static unsigned char defaultReadBufferSize = 255;
};
