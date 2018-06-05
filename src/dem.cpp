#include "dem.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <algorithm>

/**
 * @brief DEM::DEM: constructor
 * @param dem_config: pointer of the configure
 * @param flags: use linear or gaussian interpolation
 */
DEM::DEM(DEMConfig_ptr *dem_config, int flags)
{
    memcpy(&config,dem_config,sizeof(DEMConfig));
    inter_type = flags;
    variableInit();
}

DEM::~DEM()
{
    variableFree();
}

void DEM::variableInit()
{
    grid_data = (GridPoint_ptr*)malloc(sizeof(GridPoint_ptr) * config.size_y);
    for(int i = 0; i < config.size_y; i++)
    {
        grid_data[i] = (GridPoint_ptr)malloc(sizeof(GridPoint) * config.size_x);
    }
    for(int i=0;i<config.size_y;i++)
    {
        for(int j=0;j<config.size_x;j++)
        {
            grid_data[i][j].z_min = DBL_MAX;
            grid_data[i][j].z_max = DBL_MIN;
            grid_data[i][j].z_mean = 0;
            grid_data[i][j].z_idw = 0;
            grid_data[i][j].count = 0;
            grid_data[i][j].sum = 0;
            grid_data[i][j].empty = 1;
            grid_data[i][j].filled = 0;
        }
    }
}

void DEM::variableFree()
{
    for(int i = 0; i < config.size_y; i++)
        delete grid_data[i];
    delete grid_data;
}

/**
 * @brief DEM::update
 * @param data_x: point x
 * @param data_y: point y
 * @param data_z: point z
 * @return  -1,the point is not in the dem range; 0 correct.
 */
int DEM::update(double data_x, double data_y, double data_z)
{
    double x;
    double y;
    int lower_grid_x;
    int lower_grid_y;
    //stp1.calculate the projecting grid index
    lower_grid_x = floor(data_x/config.res_x) + config.offset_x;
    lower_grid_y = floor(data_y/config.res_y) + config.offset_y;

    if(lower_grid_x >= config.size_x || lower_grid_y >= config.size_y||
            lower_grid_x<0 || lower_grid_y<0)
    {
        return -1;
    }
    //stp2.the distance to the bottom grid
    x = (data_x - (lower_grid_x - config.offset_x) * config.res_x);
    y = (data_y - (lower_grid_y - config.offset_y) * config.res_y);
    //stp3.update all the four quadrants respectly
    updateFirstQuadrant(data_z, lower_grid_x+1, lower_grid_y+1, config.res_x - x, config.res_y - y);
    updateSecondQuadrant(data_z, lower_grid_x, lower_grid_y+1, x, config.res_y - y);
    updateThirdQuadrant(data_z, lower_grid_x, lower_grid_y, x, y);
    updateFourthQuadrant(data_z, lower_grid_x+1, lower_grid_y, config.res_x - x, y);
    return 0;
}


/**
 * @brief DEM::updateFirstQuadrant: update grid values in the first quadrant
 * @param data_z:  the point height value
 * @param first_grid_idx: the loop starting head, grid x index
 * @param first_grid_idy:  the loop starting head, grid y index
 * @param initial_dist_x:  the initial x distance of the starting head grid
 * @param initial_dist_y:  the initial y distance of the starting head grid
 */
void DEM::updateFirstQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y)
{
    for(int i = first_grid_idx; i < config.size_x; i++)
    {
        for(int j = first_grid_idy; j < config.size_y; j++)
        {
            double distance = 	pow((i - first_grid_idx)*config.res_x + initial_dist_x,2) +
                                pow(((j - first_grid_idy)*config.res_y + initial_dist_y),2);

            if(distance <= config.radius_sqr)
            {
                // update DEM_GRID
                if(inter_type==LINEAR_INTERPOLATION)
                {
                    linearGridPoint(i,j,data_z,distance);
                }
                else if(inter_type==GUASSIAN_INTERPOLATION)
                {
                    guassGridPoint(i,j,data_z,distance);
                }

            }
            else
            {
                return;
            }
        }
    }

}
void DEM::updateSecondQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y)
{
    for(int i = first_grid_idx; i >= 0; i--)
    {
        for(int j = first_grid_idy; j < config.size_y; j++)
        {
            double distance = 	pow((first_grid_idx - i)*config.res_x + initial_dist_x,2) +
                                pow((j - first_grid_idy)*config.res_y + initial_dist_y,2);

            if(distance <= config.radius_sqr)
            {
                // update DEM_GRID
                if(inter_type==LINEAR_INTERPOLATION)
                {
                    linearGridPoint(i,j,data_z,distance);
                }
                else if(inter_type==GUASSIAN_INTERPOLATION)
                {
                    guassGridPoint(i,j,data_z,distance);
                }
            }
            else
            {
                return;
            }
        }
    }

}
void DEM::updateThirdQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y)
{
    for(int i = first_grid_idx; i >= 0; i--)
    {
        for(int j = first_grid_idy; j >= 0; j--)
        {
            double distance = 	pow((first_grid_idx - i)*config.res_x + initial_dist_x,2) +
                                pow((first_grid_idy - j)*config.res_y + initial_dist_y,2);

            if(distance <= config.radius_sqr)
            {
                // update DEM_GRID
                if(inter_type==LINEAR_INTERPOLATION)
                {
                    linearGridPoint(i,j,data_z,distance);
                }
                else if(inter_type==GUASSIAN_INTERPOLATION)
                {
                    guassGridPoint(i,j,data_z,distance);
                }
            }
            else
            {
                return;
            }
        }
    }
}
void DEM::updateFourthQuadrant(double data_z, int first_grid_idx, int first_grid_idy, double initial_dist_x, double initial_dist_y)
{
    for(int i = first_grid_idx; i < config.size_x; i++)
    {
        for(int j = first_grid_idy; j >= 0; j--)
        {
            double distance = 	pow((i - first_grid_idx)*config.res_x + initial_dist_x,2) +
                                pow((first_grid_idy - j)*config.res_y + initial_dist_y,2);

            if(distance <= config.radius_sqr)
            {
                // update DEM_GRID
                if(inter_type==LINEAR_INTERPOLATION)
                {
                    linearGridPoint(i,j,data_z,distance);
                }
                else if(inter_type==GUASSIAN_INTERPOLATION)
                {
                    guassGridPoint(i,j,data_z,distance);
                }
            }
            else
            {
                return;
            }
        }
    }
}

/**
 * @brief DEM::linearGridPoint: linear interpolate the dem
 * @param grid_idx: index x grid
 * @param grid_idy: index y grid
 * @param data_z:  the data_z of surroundings
 * @param distance: the distance between data_z and the exact grid
 */
void DEM::linearGridPoint(int grid_idx, int grid_idy, double data_z, double distance)
{
    //stp1. hypothesis that the z of each grid is the same as its surroundings
    if(grid_data[grid_idy][grid_idx].z_min > data_z)
    {
        grid_data[grid_idy][grid_idx].z_min = data_z;
    }
    if(grid_data[grid_idy][grid_idx].z_max < data_z)
    {
        grid_data[grid_idy][grid_idx].z_max = data_z;
    }
    grid_data[grid_idy][grid_idx].z_mean += data_z;
    grid_data[grid_idy][grid_idx].count++;

    //stp2. hypothesis that the z of each grid is the distribution of its surroundings
    double dist = pow(sqrt(distance), config.linear_weight);
    if(distance != 0)
    {
        grid_data[grid_idy][grid_idx].z_idw += data_z/dist;
        grid_data[grid_idy][grid_idx].sum += 1/dist;
    }
    else
    {
        grid_data[grid_idy][grid_idx].z_idw = data_z;
        grid_data[grid_idy][grid_idx].sum = -1;
    }
}

void DEM::guassGridPoint(int grid_idx, int grid_idy, double data_z, double distance)
{
    if(grid_data[grid_idy][grid_idx].z_min > data_z)
    {
        grid_data[grid_idy][grid_idx].z_min = data_z;
    }
    if(grid_data[grid_idy][grid_idx].z_max < data_z)
    {
        grid_data[grid_idy][grid_idx].z_max = data_z;
    }
    grid_data[grid_idy][grid_idx].z_mean += data_z;
    grid_data[grid_idy][grid_idx].count++;

    double dist = exp(-0.5*distance/(config.guasian_sigma*config.guasian_sigma));
    if(distance != 0)
    {
        grid_data[grid_idy][grid_idx].z_idw += data_z*dist;
        grid_data[grid_idy][grid_idx].sum += dist;
    }
    else
    {
        grid_data[grid_idy][grid_idx].z_idw = data_z;
        grid_data[grid_idy][grid_idx].sum = -1;
    }
}

void DEM::finish()
{
    for(int i = 0; i < config.size_y; i++)
    {
        for(int j = 0; j < config.size_x; j++)
        {
            if(grid_data[i][j].z_min == DBL_MAX)
            {
                grid_data[i][j].z_min = 0;
            }

            if(grid_data[i][j].z_max == -DBL_MAX)
            {
                grid_data[i][j].z_max = 0;
            }

            if(grid_data[i][j].count != 0)
            {
                grid_data[i][j].z_mean /= grid_data[i][j].count;
                grid_data[i][j].empty = 0;
                grid_data[i][j].filled = 1;
            }
            else
            {
                grid_data[i][j].z_mean = 0;
            }

            if(grid_data[i][j].sum != 0 && grid_data[i][j].sum != -1)
            {
                grid_data[i][j].z_idw /= grid_data[i][j].sum;
            }
            else if (grid_data[i][j].sum == -1)
            {
                // do nothing
            }
            else
            {
                grid_data[i][j].z_idw = 0;
            }
        }
    }
    if(inter_type==LINEAR_INTERPOLATION)
    {
        linearFilter();
    }
    else if(inter_type==GUASSIAN_INTERPOLATION)
    {
        guassFilter();
    }
    else
    {
    }
}

void DEM::guassFilter()
{
    int window_size = config.window_size;
    if (window_size == 0)
    {
        printf("WRN:DEM Filter window size=0\n");
        return;
    }
    int window_dist = window_size / 2;
    for (int i = 0; i < config.size_y; i++)
    {
        for (int j = 0; j < config.size_x; j++)
        {
            if (grid_data[i][j].empty == 1)
            {
                double new_sum=0.0;
                for (int p = i - window_dist; p <= i + window_dist; p++)
                {
                    for (int q = j - window_dist; q <= j + window_dist; q++)
                    {
                        if ((p >= 0) && (p < config.size_y) && (q >=0) && (q < config.size_x))
                        {
                            if (grid_data[p][q].empty == 0)
                            {
                                double distance = exp(-0.5*(pow((p-i)*config.res_y,2)+pow((q-j)*config.res_x,2))
                                                            / (config.guasian_sigma*config.guasian_sigma));
                                grid_data[i][j].z_mean += grid_data[p][q].z_mean * distance;
                                grid_data[i][j].z_idw += grid_data[p][q].z_idw * distance;
                                grid_data[i][j].z_min += grid_data[p][q].z_min * distance;
                                grid_data[i][j].z_max += grid_data[p][q].z_max* distance;
                                new_sum += distance;
                            }
                        }
                    }
                }
                if (new_sum > 0) {
                    grid_data[i][j].z_mean /= new_sum;
                    grid_data[i][j].z_idw /= new_sum;
                    grid_data[i][j].z_min /= new_sum;
                    grid_data[i][j].z_max /= new_sum;
                    grid_data[i][j].filled = 1;
                    grid_data[i][j].empty = 0;
                }
            }
        }
    }
}

void DEM::linearFilter()
{
    int window_size = config.window_size;
    if (window_size == 0)
    {
        printf("WRN:DEM Filter window size=0\n");
        return;
    }
    int window_dist = window_size / 2;
    for (int i = 0; i < config.size_y; i++)
    {
        for (int j = 0; j < config.size_x; j++)
        {
            if (grid_data[i][j].empty == 1)
            {
                double new_sum=0.0;
                for (int p = i - window_dist; p <= i + window_dist; p++)
                {
                    for (int q = j - window_dist; q <= j + window_dist; q++)
                    {
                        if ((p >= 0) && (p < config.size_y) && (q >=0) && (q < config.size_x))
                        {
                            if (grid_data[p][q].empty == 0)
                            {
                                //in the original source code,the max distance between x gap and y gap is chosen as the weight,
                                // maybe it can also be  calculated by the Eular distance
                                double distance = pow(std::max(fabs(p-i), fabs(q-j)),config.linear_weight);
                                grid_data[i][j].z_mean += grid_data[p][q].z_mean/distance;
                                grid_data[i][j].z_idw += grid_data[p][q].z_idw/distance;
                                grid_data[i][j].z_min += grid_data[p][q].z_min/distance;
                                grid_data[i][j].z_max += grid_data[p][q].z_max/distance;
                                new_sum += 1/distance;
                            }
                        }
                    }
                }
                if (new_sum > 0) {
                    grid_data[i][j].z_mean /= new_sum;
                    grid_data[i][j].z_idw /= new_sum;
                    grid_data[i][j].z_min /= new_sum;
                    grid_data[i][j].z_max /= new_sum;
                    grid_data[i][j].filled = 1;
                    grid_data[i][j].empty = 0;
                }
            }
        }
    }
}





