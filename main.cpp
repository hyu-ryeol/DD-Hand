
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
#include "Poco/Net/ServerSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Timespan.h"
#include "Poco/NumericString.h"
#include <iostream>
#include <time.h>
#include <signal.h>
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




union RecvData
{
    unsigned char byte[10];
    unsigned int cmd[5];
};


float T_kp[4] = {0,0,15,0};
float I_kp[4] = {0,0,0,0};
float M_kp[4] = {0,0,0,0};
float R_kp[4] = {0,0,0,0};
float P_kp[2] = {0,0};
//float T_kd[4] = {0,0,0,0};
//float I_kd[4] = {0,0,0,0};
//float M_kd[4] = {0,0,0,0};
//float R_kd[4] = {0,0,0,0};



float T_kd[4] = {0,0,0.75,0};
float I_kd[4] = {0,0,0,0};
float M_kd[4] = {0,0,0,0};
float R_kd[4] = {0,0,0,0};
float P_kd[2] = {0,0};



uint16_t T_target[4] = {30000,22000,49700,15300};
uint16_t I_target[4] = {24000,26000,50000,15300};
uint16_t M_target[4] = {30000,23000,49000,15300};
uint16_t R_target[4] = {24000,26500,32000,15300};
uint16_t P_target[2] = {25000,25000};


//
//uint16_t T_target[4] = {0x000, 0x000, 0x000, 0x000};
//uint16_t I_target[4] = {0x000, 0x000, 0x000, 0x000};
//uint16_t M_target[4] = {0x000, 0x000, 0x000, 0x000};
//uint16_t R_target[4] = {0x000, 0x000, 0x000, 0x000};

volatile uint16_t T_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t I_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t M_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t R_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t P_pos[2] = {0x8000,0x8000};

uint16_t preT_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preI_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preM_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preR_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preP_pos[2] = {0x8000,0x8000};

uint16_t preT_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preI_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preM_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preR_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preP_err[2] = {0x8000,0x8000};



int T_Err[4] = {0,0,0,0};
int I_Err[4] = {0,0,0,0};
int M_Err[4] = {0,0,0,0};
int R_Err[4] = {0,0,0,0};
int P_Err[2] =  {0,0};

float T_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float I_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float M_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float R_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float P_torque[4] = {0x8000,0x8000};

uint16_t offset = 0x8000;
uint8_t dataDivide_flag = 0; // 0: motor , 1: sensor

bool rxSuccess_flag = false;
bool prePos_flag = false;


const std::string hostname = "169.254.186.72"; //STEP2 IP Address

const Poco::UInt16 PORT = 7;
StreamSocket ss;
enum {
    SIZE_HEADER = 52,
    SIZE_COMMAND = 4,
    SIZE_HEADER_COMMAND = 56,
    SIZE_DATA_MAX = 37,
    SIZE_DATA_ASCII_MAX = 37
};

void ctrlchandler(int)
{
    unsigned char writeBuff[37];
    writeBuff[0] = 0x00;
    for(int i=0 ; i<18 ; i++){
        writeBuff[i*2+1] = 0x80;
        writeBuff[i*2+2] = 0x00;
    }
    ss.sendBytes(writeBuff,37,0);
    usleep(2000);
    ss.sendBytes(writeBuff,37,0);
    usleep(2000);
    ss.close();
    cout << "EXIT..." << endl;

    exit(EXIT_SUCCESS);
}

void killhandler(int)
{
    unsigned char writeBuff[37];
    writeBuff[0] = 0x00;
    for(int i=0 ; i<18 ; i++){
        writeBuff[i*2+1] = 0x80;
        writeBuff[i*2+2] = 0x00;
    }
    ss.sendBytes(writeBuff,37,0);
    usleep(2000);
    ss.sendBytes(writeBuff,37,0);
    usleep(2000);
    ss.close();

    cout << "EXIT..." << endl;

    exit(EXIT_SUCCESS);
}





class HYUControl: public Poco::Runnable{
    virtual void run(){
        unsigned char writeBuff[37];
        writeBuff[0] = 0x00;
        for(int i=0 ; i<18 ; i++){
            writeBuff[i*2+1] = 0x80;
            writeBuff[i*2+2] = 0x00;
        }

        ss.sendBytes(writeBuff,37,0);
	int toggle = 1;
	unsigned int count = 0;
        while(1){
		count ++;
		if(count >500){
			count = 0;
			if (toggle ==1){
				M_target[0] = 4000;
				I_target[0] = 4000;
				T_target[0] = 4000;
			}
			else{
				M_target[0] = 30000;
				I_target[0] = 30000;
				T_target[0] = 30000;
			}
			toggle *=-1;
		}
            unsigned char receiveBuff[37];
	    ss.receiveBytes(receiveBuff,37,0);
            for(int i=0 ; i<4 ; i++){ // 32/2/4 -> 32/8 -> 4
                T_pos[i] = ((receiveBuff[i*2] << 8) & 0xFF00) | (receiveBuff[i*2+1] & 0x00FF);
                I_pos[i] = ((receiveBuff[i*2+8] << 8) & 0xFF00) | (receiveBuff[i*2+9] & 0x00FF);
                M_pos[i] = ((receiveBuff[i*2+16] << 8) & 0xFF00) | (receiveBuff[i*2+17] & 0x00FF);
                R_pos[i] = ((receiveBuff[i*2+24] << 8) & 0xFF00) | (receiveBuff[i*2+25] & 0x00FF);
            }
	    for(int i =0;i<2;i++){
	    	P_pos[i] = ((receiveBuff[i*2+32] << 8) & 0xFF00) | (receiveBuff[i*2+33]& 0x00FF);
	    }
            for(int i=0; i<4; i++){
                T_Err[i] = T_target[i] - T_pos[i];
                I_Err[i] = I_target[i] - I_pos[i];
                M_Err[i] = M_target[i] - M_pos[i];
                R_Err[i] = R_target[i] - R_pos[i];
            }
	    for(int i = 0;i<2;i++){
		P_Err[i] = P_target[i] - P_pos[i];
	    
	    }
                cout << "\x1B[2J\x1B[H";

	    cout<<"T_Err : "<<T_Err[0]<<","<<T_Err[1]<<","<<T_Err[2]<<","<<T_Err[3] <<std::endl;
             cout<<"T_preErr : "<< (T_pos[0]-preT_pos[0])<<","<< (T_pos[1]-preT_pos[1])<<","<< (T_pos[2]-preT_pos[2])<<","<< (T_pos[3]-preT_pos[3]) <<std::endl;
	    cout<<"T_pos : "<<T_pos[0]<<","<<T_pos[1]<<","<<T_pos[2]<<","<<T_pos[3] <<std::endl;
	    cout<<"preT_pos : "<<preT_pos[0]<<","<<preT_pos[1]<<","<<preT_pos[2]<<","<<preT_pos[3]<<std::endl;
           // cout<<"I_Err : "<<I_Err[0]<<","<<I_Err[1]<<","<<I_Err[2]<<","<<I_Err[3] <<endl;
           // cout<<"M_Err : "<<M_Err[0]<<","<<M_Err[1]<<","<<M_Err[2]<<","<<M_Err[3] <<endl;
           // cout<<"R_Err : "<<R_Err[0]<<","<<R_Err[1]<<","<<R_Err[2]<<","<<R_Err[3] <<endl;
            for(int i=0; i<4; i++){
              //  T_torque[i] = (float)(T_kp[i]*(T_Err[i])+T_kd[i]*(float)(-1*(T_pos[i]-preT_pos[i])/0.002))+offset; //offset = 0x8000
              // T_torque[i] = (10000.0*(float)(-1.0*(T_pos[i]-preT_pos[i])))+offset; //offset = 0x8000
		I_torque[i] = (float)(I_kp[i]*(I_Err[i])+I_kd[i]*(float)(-1*(I_pos[i]-preI_pos[i])/0.002))+offset;
                M_torque[i] = (float)(M_kp[i]*(M_Err[i])+M_kd[i]*(float)(-1*(M_pos[i]-preM_pos[i])/0.002))+offset;
                R_torque[i] = (float)(R_kp[i]*(R_Err[i])+R_kd[i]*(float)(-1*(R_pos[i]-preR_pos[i])/0.002))+offset;
                if(T_torque[i]<0) T_torque[i] = 0;
                if(I_torque[i]<0) I_torque[i] = 0;
                if(M_torque[i]<0) M_torque[i] = 0;
                if(R_torque[i]<0) R_torque[i] = 0;

                if(T_torque[i]>65534) T_torque[i] = 65534;
                if(I_torque[i]>65534) I_torque[i] = 65534;
                if(M_torque[i]>65534) M_torque[i] = 65534;
                if(R_torque[i]>65534) R_torque[i] = 65534;
		 
            }
            for(int i = 0;i<2;i++){
	    	P_torque[i] = (float)(P_kp[i]*(P_Err[i])+P_kd[i]*(-1*(P_pos[i]-preP_pos[i])/0.002)) + offset;
		if(P_torque[i]<0)P_torque[i] = 0;
		if(P_torque[i]>65534)P_torque[i]=65534;
	    	
	    }
	     cout<<"T_torque : "<<T_torque[0]<<","<<T_torque[1]<<","<<T_torque[2]<<","<<T_torque[3] <<std::endl;
            for(int i=0 ; i<4 ; i++){
                preT_pos[i] = T_pos[i];
                preI_pos[i] = I_pos[i];
                preM_pos[i] = M_pos[i];
                preR_pos[i] = R_pos[i];

            }
	    for(int i=0;i<2;i++){
	    	preP_pos[i] = P_pos[i];

	    }


            unsigned char TIMR_Duty[37];
            TIMR_Duty[0] = 0x00; // ID
            for(int i=0; i<4; i++){
                TIMR_Duty[i*2+1] = ((int)T_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+2] = (int)T_torque[i] & 0x00FF;

                TIMR_Duty[i*2+9] = ((int)I_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+10] = (int)I_torque[i] & 0x00FF;

                TIMR_Duty[i*2+17] = ((int)M_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+18] = (int)M_torque[i] & 0x00FF;

                TIMR_Duty[i*2+25] = ((int)R_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+26] = (int)R_torque[i] & 0x00FF;
            }
            for(int i =0;i<2;i++){
	    	TIMR_Duty[i*2+33] =  ((int)P_torque[i] >> 8 ) & 0x00FF;
		TIMR_Duty[i*2+34] = (int)P_torque[i] & 0x00FF;
	    }

            ss.sendBytes(TIMR_Duty,37,0);
        }

    }

};
union Data
        {
    unsigned char byte[SIZE_DATA_MAX];
        };

int main(int argc, char **argv)
{

    Data data_rev;


    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    HYUControl hyu;
    Poco::Thread thread;

    try
    {
        cout << "Trying to connect Hand server..." << endl;
        ss.connect(SocketAddress(hostname, PORT));
        Timespan timeout(1, 0);
        while (ss.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == false)
        {
            cout << "Connecting to Hand server..." << endl;
        }


        cout << "Trying to connect Hand server..." << endl;

        thread.start(hyu);
        int count = 0;

        





        Poco::Net::SocketAddress server_addr(9911);
        Poco::Net::ServerSocket server_sock(server_addr);
        Poco::Net::Socket::SocketList connectedSockList;

        connectedSockList.push_back(server_sock);
        RecvData data_rev;
            cout << "TCP Server is on" << endl;
        
        while (true){
            Poco::Net::Socket::SocketList readList(connectedSockList.begin(), connectedSockList.end());
            Poco::Net::Socket::SocketList writeList(connectedSockList.begin(), connectedSockList.end());
            Poco::Net::Socket::SocketList exceptList(connectedSockList.begin(), connectedSockList.end());
            Poco::Timespan timeout(1);
            auto count = Poco::Net::Socket::select(readList, writeList, exceptList, timeout);
            if (count == 0)
                continue;

            Poco::Net::Socket::SocketList delSockList;
            for (auto& readSock : readList)
                {
                    if (server_sock == readSock)
                    {
                        auto newSock = server_sock.acceptConnection();
                        connectedSockList.push_back(newSock);
                        std::cout << "NEW CLIENT CONNECTED" << std::endl;
                    }
                    else
                    {
                        char buffer[10] = { 0 };
                        auto n = ((Poco::Net::StreamSocket*)&readSock)->receiveBytes(buffer, 10);
                        if (n > 0)
                        {
                            std::cout << "RECEIVED MESSEGE: " <<buffer << std::endl;
                            memcpy(data_rev.byte, buffer, SIZE_DATA_MAX);
                            std::cout << data_rev.cmd[0]<<std::endl;
                            if ( data_rev.cmd[0] == 1){
                                 std::cout << "MOTION 1 " <<buffer << std::endl;
                                T_target[0] = 34000;
                                T_target[1] = 34000;
                                T_target[2] = 32768;
                                T_target[3] = 32768;

                                I_target[0] = 32768;
                                I_target[1] = 32768;
                                I_target[2] = 32768;
                                I_target[3] = 28000;

                                M_target[0] = 32768;
                                M_target[1] = 32768;
                                M_target[2] = 32768;
                                M_target[3] = 32768;

                                R_target[0] = 32768;
                                R_target[1] = 32768;
                                R_target[2] = 32768;
                                R_target[3] = 38000;
                            }
                            else if ( data_rev.cmd[0] == 2){
                                 std::cout << "MOTION 2  "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 40000;
                                T_target[1] = 5000;
                                T_target[2] = 33000;
                                T_target[3] = 33000;

                                I_target[0] = 38000;
                                I_target[1] = 11000;
                                I_target[2] = 25000;
                                I_target[3] = 32768;

                                M_target[0] = 32768;
                                M_target[1] = 11000;
                                M_target[2] = 25000;
                                M_target[3] = 32768;

                                R_target[0] = 27000;
                                R_target[1] = 5000;
                                R_target[2] = 25000;
                                R_target[3] = 32768;
                            }
                            else if ( data_rev.cmd[0] == 3){
                                 std::cout << "MOTION 3 "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 42000;
                                T_target[1] = 35000;
                                T_target[2] = 30000;
                                T_target[3] = 13000;

                                I_target[0] = 28000;
                                I_target[1] = 19000;
                                I_target[2] = 27000;
                                I_target[3] = 55000;

                                M_target[0] = 32768;
                                M_target[1] = 19000;
                                M_target[2] = 27000;
                                M_target[3] = 55000;

                                R_target[0] = 38000;
                                R_target[1] = 15000;
                                R_target[2] = 27000;
                                R_target[3] = 58000;
                            }

                            else if ( data_rev.cmd[0] == 4){
                                 std::cout << "MOTION 4 "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 42000;
                                T_target[1] = 500;
                                T_target[2] = 30000;
                                T_target[3] = 13000;

                                I_target[0] = 28000;
                                I_target[1] = 19000;
                                I_target[2] = 27000;
                                I_target[3] = 53000;

                                M_target[0] = 32768;
                                M_target[1] = 19000;
                                M_target[2] = 27000;
                                M_target[3] = 53000;

                                R_target[0] = 38000;
                                R_target[1] = 16000;
                                R_target[2] = 27000;
                                R_target[3] = 53000;
                            }

                            else if ( data_rev.cmd[0] == 5){
                                 std::cout << "MOTION 5 "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 33000;
                                T_target[1] = 1000;
                                T_target[2] = 12000;
                                T_target[3] = 12000;

                                I_target[0] = 30000;
                                I_target[1] = 32768;
                                I_target[2] = 32768;
                                I_target[3] = 32768;

                                M_target[0] = 32768;
                                M_target[1] = 12000;
                                M_target[2] = 12000;
                                M_target[3] = 50000;

                                R_target[0] = 32768;
                                R_target[1] = 12000;
                                R_target[2] = 12000;
                                R_target[3] = 50000;
                            }
                            else if ( data_rev.cmd[0] == 6){
                                 std::cout << "MOTION 6 "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 33000;
                                T_target[1] = 1000;
                                T_target[2] = 12000;
                                T_target[3] = 12000;

                                I_target[0] = 30000;
                                I_target[1] = 12000;
                                I_target[2] = 12000;
                                I_target[3] = 55000;

                                M_target[0] = 32768;
                                M_target[1] = 12000;
                                M_target[2] = 12000;
                                M_target[3] = 55000;

                                R_target[0] = 32768;
                                R_target[1] = 12000;
                                R_target[2] = 12000;
                                R_target[3] = 50000;
                            }
                            std::cout << "-------------------------" <<std::endl;
                        }
                        else
                        {
                            std::cout << "클라이언트와 연결이 끊어졌습니다." << std::endl;
                            delSockList.push_back(readSock);
                        }
                    }
                }
         
                for (auto& delSock : delSockList)
                {
                    // 삭제할 소켓을 검색한다.
                    auto delIter = std::find_if(connectedSockList.begin(),
                        connectedSockList.end(),
                        [&delSock](auto& sock)
                        {
                            return delSock == sock ? true : false;
                        }
                    );
         
                    if (delIter != connectedSockList.end())
                    {
                        connectedSockList.erase(delIter);
                        std::cout << "connectedSockList 에서 socket 제거" << std::endl;
                    }
                }
        }






    }
    catch (Poco::Exception& exc)
    {
        cout << "Fail to connect server..." << exc.displayText() << endl;
    }
    ss.close();
    thread.join();


    return 0;
}



