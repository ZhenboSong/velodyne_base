#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include "common.h"
#include "velo_driver.h"


/** @brief constructor
 *  @param device IP in char array
 *  @param data port
 */
VeloDriver::VeloDriver(const char *device_ip, const unsigned int data_port)
{
    //stp1. init the member variables
    variableInit();
    //stp2. communicate with device
    startComm(device_ip,data_port);
    //stp3. start recv thread
    recv_thread_handle = std::thread(recvThread,this);
    recv_thread_handle.detach();
}

VeloDriver::~VeloDriver()
{
    //stp1. free variables
    variableFree();
    //stp2. close socket
    (void)close(sock_fd);
}

void VeloDriver::variableInit()
{
    sock_fd = -1;
    frame_id = 0;
    curr_rot_ang = 0;
    last_rot_ang = 0;

    pthread_mutex_init(&pack_lock,nullptr);
    pthread_cond_init(&pack_new_signal,nullptr);

    recv_data = new FrameData;
    pass_data = new FrameData;
    raw_data = new FrameData;

    memset(recv_data,0,sizeof(FrameData));
    memset(pass_data,0,sizeof(FrameData));
    memset(raw_data,0,sizeof(FrameData));
}

void VeloDriver::variableFree()
{
    if(recv_data)
    {
        delete recv_data;
    }
    if(pass_data)
    {
        delete pass_data;
    }
    if(raw_data)
    {
        delete raw_data;
    }
}

void VeloDriver::startComm(const char * device_ip,const unsigned int data_port)
{
    //open the socket and bind with the device ip
    if (device_ip!=nullptr)
    {
        inet_aton(device_ip,&dev_ip);
    }
    sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock_fd == -1)
    {
        perror("socket");
        return;
    }
    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(data_port);     // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP
    if (bind(sock_fd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
    {
        perror("bind");
        return;
    }
    if (fcntl(sock_fd,F_SETFL, O_NONBLOCK|FASYNC) < 0)
    {
        perror("non-block");
        return;
    }
}

int VeloDriver::newData()
{
    memset(raw_data,0,sizeof(FrameData));
    //TODO:modify into timedwait and multiple return values
    pthread_mutex_lock(&pack_lock);
    pthread_cond_wait(&pack_new_signal,&pack_lock);
    memcpy(raw_data,pass_data,sizeof(FrameData));
    pthread_mutex_unlock(&pack_lock);
    memset(pass_data,0,sizeof(FrameData));
    return 1;
}

void VeloDriver::recvThread(void * arg)
{
    VeloDriver * p_this = (VeloDriver*) arg;
    while(true)
    {
        int p_key = p_this->getPacket();
        if(p_key!=0)
        {
            break;
        }
    }
    return;
}

int VeloDriver::getPacket()
{
    //stp1.init socket variables for timeout function
    struct pollfd fds[1];
    fds[0].fd = sock_fd;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1*1000; // 120 seconds (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    //stp2.init the variables in intermediate process
    char buff[2048];
    while (true)
    {
        //stp3-1. timeout function
        // the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.
        // poll() until input available
        do
        {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
            {
                if (errno != EINTR)
                    perror("poll() error");
                return 1;
            }
            if (retval == 0)            // poll() timeout?
            {
                printf("WRN:Velodyne poll() timeout\n");
                return 0;
            }
            if ((fds[0].revents & POLLERR)
                    || (fds[0].revents & POLLHUP)
                    || (fds[0].revents & POLLNVAL)) // device error?
            {
                printf("ERRO:poll() reports Velodyne error\n");
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

        //stp3-2.Receive packets that should now be available from the
        //socket using a blocking read.
        memset(buff,0,2048);
        ssize_t nbytes = recvfrom(sock_fd, buff, 2048,  0,
                                  (sockaddr*) &sender_address,
                                  &sender_address_len);
        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                return 1;
            }
            break;
        }
        else
        {
            analysePacket(buff,nbytes);
            if(sender_address.sin_addr.s_addr != dev_ip.s_addr)
            {
                printf("WRN:recv ip is not the same with device ip\n");
                continue;
            }
            else
            {
                //printf("LOG:recv ok\n");
                break;
            }
            break;
        }
    }
    return 0;
}

/** @brief analyse the UDP packet from the lidar device
 *  @param buffer address
 *  @param length of the receiving buffer
 */
void VeloDriver::analysePacket(char* buf, int len)
{
    if(len != PACKET_SIZE)
    {
        printf("WRN:package erro, not the correct size\n");
        return;
    }
    unsigned char * p_data = (unsigned char *)buf;
    unsigned int temp_time_stampe = p_data[1200] + (p_data[1201]<<8) + (p_data[1202]<<16) + (p_data[1203]<<24);  //10E-6 second
    unsigned char temp_status_type = (unsigned char)p_data[1204];
    unsigned char temp_status_value = (unsigned char)p_data[1205];
    int block_index = 0;
    int laser_index = 0;
    //every packet have 12 blocks
    while(block_index < 12)
    {
        //100 * degree of the scan angle, 0-36000
        curr_rot_ang = (p_data[3]<<8) + p_data[2];
        if(last_rot_ang != 0&&curr_rot_ang >= 18000 && last_rot_ang < 18000)
        {
            //cut the continuous data into frames at the 180 degree point
            recv_data->frame_id = frame_id;
            pthread_mutex_lock(&pack_lock);
            memcpy(pass_data,recv_data,sizeof(FrameData));
            pthread_cond_signal(&pack_new_signal);
            pthread_mutex_unlock(&pack_lock);
            frame_id ++;
            memset(recv_data,0,sizeof(FrameData));
        }

        //resolve the raw data into FrameData struct
        recv_data->frame_block[recv_data->block_num].upper_or_lower = (p_data[1]<<8) + p_data[0];
        recv_data->frame_block[recv_data->block_num].block_id = block_index;
        recv_data->frame_block[recv_data->block_num].rot_angle = curr_rot_ang;
        p_data += 4;
        while(laser_index < 32)
        {
            //length of the scanning measure, (*2 mm)
            recv_data->frame_block[recv_data->block_num].fire_laser[laser_index].distance = ((p_data[1] << 8) + p_data[0]);
            recv_data->frame_block[recv_data->block_num].fire_laser[laser_index].intensity = p_data[2];
            laser_index ++;
            p_data += 3;
        }
        //TODO: analyse the timestamp of every block
        recv_data->frame_block[recv_data->block_num].gps_time_stampe = temp_time_stampe;
        recv_data->frame_block[recv_data->block_num].gps_status_type = temp_status_type;
        recv_data->frame_block[recv_data->block_num].gps_status_value = temp_status_value;
        recv_data->block_num ++;
        block_index ++;
        laser_index = 0;
        last_rot_ang = curr_rot_ang;
    }
}

