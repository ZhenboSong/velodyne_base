#include "velo_driver.h"
#include <iostream>
#include <time.h>
using namespace std;

int main()
{
    char device_ip[128] = "192.168.2.201";
    unsigned int data_port = 2368;
    VeloDriver velo64_driver(device_ip,data_port);
    char save_file_dir[128];
    while(velo64_driver.newData())
    {
        sprintf(save_file_dir,"../data/%06u.bin",velo64_driver.raw_data->frame_id);
        FILE * fp = fopen(save_file_dir,"wb");
        fwrite(velo64_driver.raw_data,sizeof(FrameData),1,fp);
        fclose(fp);
        printf("LOG:frame %u time:%lu  size:%u\n",velo64_driver.raw_data->frame_id,clock()/1000,velo64_driver.raw_data->block_num);
    }
    return 0;
}

