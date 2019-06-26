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

using namespace std;

class SerialPort
{
public:
  SerialPort(const char *device_name);
  virtual ~SerialPort(void);

  void Open(const char *device_name);
  void Close();
  void Configure();
  
  
  void Read(unsigned char* rpacket, int packetsize);
  void Read(string& data);
  void Write(unsigned char*  wpacket, int packetsize);
  void Write(const string& data);
  
private:
  int fd; // File Descriptor
  vector<char> readBuffer;
  const static unsigned char defaultReadBufferSize = 255;
};


class queue3{
    public:
        unsigned char buf[3];
        int _size;
        queue3();
        void push(unsigned char src);
        bool check_start();
        void print();
};
