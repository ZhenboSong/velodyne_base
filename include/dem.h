/**
* From Pointcloud to DEM
* last modified: 2018.6.5
*
* Zhenbo Song(songzb@njust.edu.cn)
*
* Based on the notes by Prof. Ramon Arrowsmith(ramon.arrowsmith@asu.edu)
* Authors: Han S Kim (hskim@cs.ucsd.edu), Sriram Krishnan (sriram@sdsc.edu)
* https://github.com/OpenTopography/points2grid
*
* illustration:
* stp1（setup the gridmap): DEM(DEMConfig * dem_config);
* stp2（go through all the points to generate  DEM）:for 1:size(pointcloud) int update(double data_x, double data_y, double data_z); end
* stp3（filter DEM to be smooth）: finish();
*/
#ifndef __DEM_H__
#define __DEM_H__

#pragma pack(push,1)
typedef struct tagGridPoint
{
    double z_min;
    double z_max;
    double z_mean;
    unsigned int count;
    double z_idw;
    double sum;
    int empty;
    int filled;
}GridPoint,*GridPoint_ptr;

/** Configure inparameter：
*   res_x，res_y ( x,y resolution )
*   size_x,size_y ( x,y grid number )
*   offset_x,offset_y ( x,y offset grid number )
*   window_size ( filtering window size )
*   radius_sqr ( square of the interpolation range )
*/
typedef struct tagDEMConfig
{
    double res_x;
    double res_y;
    int size_x;
    int size_y;
    int offset_x;
    int offset_y;
    int window_size;
    int linear_weight;
    double guasian_sigma;
    double radius_sqr;
}DEMConfig,*DEMConfig_ptr;

enum
{
    LINEAR_INTERPOLATION = 0,
    GUASSIAN_INTERPOLATION
};
#pragma pack(pop)

class DEM
{
public:
    //Constructor and destructor
    DEM(DEMConfig_ptr * dem_config,int flags);
     ~DEM();

    //API,member functions
    //1.update dem,input the point in each oriention
    int update(double data_x, double data_y, double data_z);
    //2. finish the dem and filter it
    void finish();

    //API, member variables
    //1.grid data,  grid_data[y_index][x_index]
    GridPoint_ptr * grid_data;


private:
    //member variables
    //1.configures of the dem
    DEMConfig  config;
    //2.type of interpolation method
    int inter_type;

    //member functions
    //1.Init all variables
    void variableInit();
    //2.Free all variables
    void variableFree();
    //3.updataDEM
    void updateFirstQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y);
    void updateSecondQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y);
    void updateThirdQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y);
    void updateFourthQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y);
    //4.calculate the weight in the grid
    void linearGridPoint(int grid_idx, int grid_idy, double data_z, double distance);
    void guassGridPoint(int grid_idx, int grid_idy, double data_z, double distance);
    //5.filter and interpolate in the grid scale
    void linearFilter();
    void guassFilter();

};

#endif
