/**
* commom structure and defines for 32E and 64E
* last modified: 2018.6.5
*
* Zhenbo Song(songzb@njust.edu.cn)
*
*/
#ifndef __COMMON_H__
#define __COMMON_H__

//depend on lidar type and rotation RPM.
//MAX_BLOCK_NUM = MAX_LINE_POINT * LASER_NUM / 32
//MAX_LINE_POINT = 2500 *  600 / RPM
//TODO: write these number into configure file; adapt the 16 vlp lidar.
#define  MAX_BLOCK_NUM       5000
#define  MAX_LINE_POINT      2500
#define  LASER_NUM           64

//Math use



#pragma pack(push)
#pragma pack(1)

//lidar raw data
//1.each measure data
typedef struct tagLaser
{
    unsigned int    distance;
    unsigned char   intensity;
}Laser,*Laser_ptr;

//2.data in every block
typedef struct tagBlock
{
    unsigned short upper_or_lower;
    unsigned short block_id;
    unsigned short rot_angle;
    Laser fire_laser[32];
    unsigned int gps_time_stampe;
    unsigned char gps_status_type;
    unsigned char gps_status_value;
}Block,*Block_ptr;

typedef struct tagFrameData
{
    unsigned int frame_id;
    unsigned int block_num;
    Block frame_block[MAX_BLOCK_NUM];
}FrameData,*FrameData_ptr;

//
typedef struct tagPoint3FI
{
    double x;
    double y;
    double z;
    unsigned char   intensity;
}Point3FI,*Point3FI_ptr;

typedef struct  tagPoint3II
{
    int x;
    int y;
    int z;
    unsigned char   intensity;
}Point3II,*Point3II_ptr;

typedef struct tagPoint2I
{
    int x;
    int y;
}Point2I,*Point2I_ptr;

typedef struct tagPoint2F
{
    double x;
    double y;
}Point2F,*Point2F_ptr;

typedef struct tagPoint3I
{
    int x;
    int y;
    int z;
}Point3I,*Point3I_ptr;

typedef struct tagPoint3F
{
    double x;
    double y;
    double z;
}Point3F,*Point3F_ptr;


typedef struct tagPoint3FIT
{
    double x;
    double y;
    double z;
    unsigned char intensity;
    unsigned int  time;
}Point3FIT,*Point3FIT_ptr;

typedef struct  tagPoint3IIT
{
    int x;
    int y;
    int z;
    unsigned char intensity;
    unsigned int  time;
}Point3IIT,*Point3IIT_ptr;

typedef struct tagPointLRDI
{
    unsigned char  line_id;
    unsigned int   rot_angle;
    unsigned int   distance;
    unsigned char  intensity;
}PointLRDI,*PointLRDI_ptr;

typedef struct tagOriginData
{
    PointLRDI  line_point[LASER_NUM][MAX_LINE_POINT];
    int line_point_num[LASER_NUM];
}OriginData,*OriginData_ptr;

typedef struct tagPointCloud
{
    Point3II line_point_cloud[LASER_NUM][MAX_LINE_POINT];
    int line_point_num[LASER_NUM];

}PointCloud,*PointCloud_ptr;



#pragma pack(pop)





#endif
