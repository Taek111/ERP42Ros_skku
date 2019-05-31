#include "SerialPort.h"
#include <ctime>

using namespace std;

SerialPort::SerialPort(const char *device_name){
  readBuffer.reserve(defaultReadBufferSize);
  Open(device_name);
}

SerialPort::~SerialPort(){
  Close();
}
void SerialPort::Open(const char *device_name){
  if (!strlen(device_name)){
    cout << "Device path has not been assigned" << endl;
    return;
  }
  fd = open(device_name, O_RDWR| O_NOCTTY);
    //O_RDWR-Read/Write access to Serial port,  O_NOCTTY -No terminal control
  if (fd == -1)
    cout << "\nError in Opening device -"<< strerror(errno) << endl;
	else
    cout << "\nDevice Opened Successfully" <<endl;
  
  // TODO: 
  // if opening device fails, Configure should not be called.
  // You have to consider the object that calls this 'Open(const char*)' function.
  // if there is some sequence that uses fd inside this object, it will fail.
  // try one of these.
  // 1. create ' bool is_open()' function and make caller recognize opening is success or not.
  // 2. make this function bool open(const char*)' and handle return value from caller
  // I recommend first one.

  Configure();
}

void SerialPort::Close(){
  if(fd != -1){
    auto retVal = close(fd);
    if(retVal != 0){
      cout << "Tried to close serial port, but close() failed " << endl;
      return;
    }
    fd = -1;
  }
}

void SerialPort::Configure(){
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if(tcgetattr(fd, &tty) != 0)
    cout << "Could not get terminal attributes - " << strerror(errno) << endl;


  tty.c_cflag &= ~PARENB;   //  No Parity  bit
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit 
  tty.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size           
  tty.c_cflag |=  CS8;      // Set the data bits = 8                                
  tty.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                       
  tty.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines 

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
  tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                           

  tty.c_oflag &= ~OPOST;//No Output Processing
  
  cfsetispeed(&tty,B9600); // Set Read  Speed as 9600                       */
  cfsetospeed(&tty,B9600); // Set Write Speed as 9600

  tty.c_cc[VMIN] = 36; // Read at least 36 characters 
  tty.c_cc[VTIME] = 0; // Wait 0.1sec  

  if((tcsetattr(fd,TCSANOW,&tty)) != 0) //Set the attributes
    cout << " Error in Setting attributes - " << strerror(errno) << endl;
  else
    cout << "BaudRate = 9600, StopBits = 1, Parity = None" << endl;
}

void SerialPort::ReadPacket(string& data){
	data.clear();
	if (fd == 0){
		cout << "read was called but file descriptor was 0, file has not been opened" << endl;
		return;
    }
    for(int i=0; i < 35; i++){
		ssize_t tmp = read(fd, &readBuffer[0], 1);
			if(tmp == -1){
				cout << "No data to read" << endl;
				return;
			}
		if(readBuffer[0] == 'X') break; 
	} 
  // TODO:
  // there is no case for fd is -1

	ssize_t n = read(fd, &readBuffer[0], 15);
	if(n != 15) {
		cout << "Packet read error!" << endl;
		return;
	}
	data = string(&readBuffer[0], n);
}

void SerialPort::Read(string& data){
	data.clear();
	if (fd == 0){
		cout << "Read was called but file descriptor was 0, file has not been opened" << endl;
		return;
    }
  // TODO:
  // there is no case for fd is -1

	ssize_t n = read(fd, &readBuffer[0], defaultReadBufferSize);
	data = string(&readBuffer[0], n);
	
}

void SerialPort::Write(const string& data){
  
  // TODO:
  // there is no case for fd is -1
int writeResult = write(fd, data.c_str(), data.size());
  
  if(writeResult == -1){
    cout << "Write failed" << endl;
  }
}


// TODO:
// test should be created on seperated files.

int test_main(void){
	
	SerialPort app = SerialPort("/dev/ttyUSB0");
	string s = "STXmEGSPSTbAECDRDA";
	string r;
	
	app.Write(s);
	app.ReadPacket(r);
	cout << r << endl;

}
