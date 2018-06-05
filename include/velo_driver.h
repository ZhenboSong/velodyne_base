/**
* Velodyne lidar Driver for 32E and 64E
* last modified: 2018.6.1
*
* Zhenbo Song(songzb@njust.edu.cn)
*
* Based on the notes ros-drivers/velodyne
* https://github.com/ros-drivers/velodyne
*/

#ifndef __VELO_DRIVER_H__
#define __VELO_DRIVER_H__

#include <netinet/in.h>
#include <thread>
#include <pthread.h>
#include "common.h"

//fix number ,no need of modifying
#define PACKET_SIZE    1206

class VeloDriver
{
public:
    //Constructor and destructor
    VeloDriver(const char * device_ip,const unsigned int data_port);
    ~VeloDriver();

    //API, member functions
    //1.the signal of whether there is a new data
    int newData();

    //API, member variables
    //1.memory for one raw lidar data frame
    FrameData_ptr raw_data;

private:
    //member variables
    //1.socket id
    int sock_fd;
    in_addr dev_ip;
    //2.recv thread id and handle
    std::thread recv_thread_handle;
    //3.thread lock ,flag and signal
    pthread_mutex_t pack_lock;
    pthread_cond_t  pack_new_signal;
    //4.current scanning angle and last scanning angle for
    //cutting data into independent frames
    unsigned short curr_rot_ang;
    unsigned short last_rot_ang;
    //5.the id number of frames
    unsigned int frame_id;
    //6.memory for save the temp data from lidar device
    FrameData_ptr recv_data;
    //7.memory for pass data between two threads
    FrameData_ptr pass_data;

    //member functions
    //1.Init all variables
    void variableInit();
    //2.Free all variables
    void variableFree();
    //3.get packet from the device
    int getPacket();
    //4.analyse every packet
    void analysePacket(char* buf, int len);
    //5.recv thread function
    static void recvThread(void *arg);
    //6.start communication with device
    void startComm(const char * device_ip,const unsigned int data_port);
};






#endif
