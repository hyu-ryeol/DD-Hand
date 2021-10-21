/* 

* TCP Example For KIST Monkey Experiment 

* TCP_HYUSPA.cpp

* Created on: Mar 2, 2020

*     Author: Sunhong Kim

*/

 

#include "Poco/Net/Net.h"

#include "Poco/Net/StreamSocket.h"

#include "Poco/Net/SocketAddress.h"

#include "Poco/Dynamic/Var.h"

#include "Poco/Exception.h"

#include "Poco/Timer.h"

#include "Poco/Stopwatch.h"

#include "Poco/Thread.h"

#include "Poco/DateTime.h"

#include "Poco/Timespan.h"

#include "Poco/NumericString.h"

#include <iostream>

#include <time.h>

 

using namespace Poco;

using namespace Poco::Dynamic;

using Poco::Net::SocketAddress;

using Poco::Net::StreamSocket;

using Poco::Net::Socket;

using Poco::Timer;

using Poco::TimerCallback;

using Poco::Thread;

using Poco::Stopwatch;

using namespace std;

 

//const std::string hostname = "127.0.0.1"; //localhost IP Address

const std::string hostname = "192.168.1.13"; //STEP2 IP Address 

//const std::string hostname = "192.168.0.100"; //STEP2 IP Address Monkey

//const std::string hostname = "192.168.1.18"; //STEP2 IP Address Tensegrity

//const std::string hostname = "192.168.0.122"; //STEP2 IP Address Tensegrity

const Poco::UInt16 PORT = 9911;

 

enum {

	SIZE_HEADER = 52,

	SIZE_COMMAND = 4,

	SIZE_HEADER_COMMAND = 56,

	SIZE_DATA_MAX = 200,

	SIZE_DATA_ASCII_MAX = 32

};

 


union Data
{
    unsigned char byte[10];
    unsigned int cmd[5];
};


 

int main()

{

	StreamSocket ss;

	int cnt = 0;

	int flag_return = 0;

	int rpt_cnt = 1;

	clock_t start, check, end, init;

	int motion_buf = 0;

	double glob_time, loop_time;

	Data data_rev, data;

	unsigned char readBuff[10];

	unsigned char writeBuff[10];

	try

	{

		cout << "Trying to connect server..." << endl;

		ss.connect(SocketAddress(hostname, PORT));

 

		Timespan timeout(1, 0);

		while (ss.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == false)

		{

			cout << "Connecting to server..." << endl;

		}

		cout << "Complete to connect server" << endl;

 

		cout << "=========== Please enter the packet ============" << endl;

		cout << "Packet scheme\n| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |" << endl;

		cout << "0 : Motion number, 1~6 : Desired data, 7 : Null" << endl;

		cout << "================================================" << endl;

 

		while (true)

		{

			cout << rpt_cnt << " : ";

			try {

				cin >> data.cmd[0];
				if(data.cmd[0] == 101){
					int toggle = 1;
					for(int j = 0;j<100;j++){
						if(toggle == 1)
							data.cmd[0] = 3;
						else if(toggle == -1) 
							data.cmd[0] = 4;
						toggle *=-1;
						memcpy(writeBuff,data.byte,SIZE_DATA_MAX);		
						ss.sendBytes(writeBuff,10);
						usleep(1000000);
					}
				}
				else{
					memcpy(writeBuff, data.byte, SIZE_DATA_MAX);

					ss.sendBytes(writeBuff, 10);
				}
			}

			catch (int expn) {

				cout << "[ERROR] : Please check the Motion" << endl;

				return 0;

			}

			rpt_cnt++;

		}

		ss.close();

	}

	catch (Poco::Exception& exc)

	{

		cout << "Fail to connect server..." << exc.displayText() << endl;

	}

	return 0;

}

 
