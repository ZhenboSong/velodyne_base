/**
* Velodyne lidar calibration for 32E and 64E
* last modified: 2018.6.5
*
* Zhenbo Song(songzb@njust.edu.cn)
*
* Based on the notes ros-drivers/velodyne
* https://github.com/ros-drivers/velodyne
*
*/

#ifndef __VELO_CLIB_H__
#define __VELO_CALIB_H__

#include "common.h"

#pragma pack(push)
#pragma pack(1)
typedef struct tagLaserCorrection {

  /** parameters in db.xml */
  double rot_correction;
  double vert_correction;
  double dist_correction;
  double dist_correction_x;
  double dist_correction_y;
  double vert_offset_correction;
  double horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  double focal_distance;
  double focal_slope;
  bool two_pt_correction_available;

  /** cached values calculated when the calibration file is read */
  double cos_rot_correction;              ///< cosine of rot_correction
  double sin_rot_correction;              ///< sine of rot_correction
  double cos_vert_correction;             ///< cosine of vert_correction
  double sin_vert_correction;             ///< sine of vert_correction
}LaserCorrection,*LaserCorrection_ptr;

#pragma pack(pop)

//fix number ,no need of modifying
#define DISTANCE_RESOLUTION 0.2f

class VeloCalib
{
public:
    //Constructor and destructor
    VeloCalib(char * calib_file_dir);
    ~VeloCalib();

    //API,member function
    //1.converse raw point into 3D point
    int convLRDI2XYZI(PointLRDI_ptr input_PointLRDI,Point3FI & output_Point3FI);
    //2.get the x'th top laser scan
    int getTopXLaser(int n){return (int)scan_table[LASER_NUM-n].x;}
    //3.get gap between the n1'th top and the n2'th top
    double getGapXYLaser(int up,int down){return (scan_table[up-1].y-scan_table[down-1].y);}

private:
    //member variables
    //1.lidar calibration file
    LaserCorrection in_param[LASER_NUM];
    //2.tables for calculating
    double cos_rot_table[36000];
    double sin_rot_table[36000];
    //3.laser scanning sorting table, bottom to top
    Point2F scan_table[LASER_NUM];

    //member functions
    //1.Init all variables
    void variableInit(){}
    //2.Free all variables
    void variableFree(){}
    //3.create calculate tables
    void createTable();
    //4.read calibration file
    int readFile(char * file_dir);
    //5.sort the scanning laser from bottom to top
    void quickSort(Point2F *s, int l, int r);
};



#endif
