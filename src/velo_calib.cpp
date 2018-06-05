#include "velo_calib.h"
#include "tinyxml2.h"
#include <math.h>

VeloCalib::VeloCalib(char *calib_file_dir)
{
    if(!readFile(calib_file_dir))
    {
        return;
    }
    variableInit();
    createTable();
    quickSort(scan_table,0,LASER_NUM-1);
//    for(int i=0;i<LASER_NUM;i++)
//    {
//        printf("laser:%d,vert:%f\n",(int)scan_table[i].x,scan_table[i].y);
//    }
}

VeloCalib::~VeloCalib()
{
    variableFree();
}

/**
 * @brief VeloCalib::convLRDI2XYZI converse raw pointLRDI( 2mm )  into Point3FI(cm), calculta in (cm)
 * @param input_PointLRDI
 * @param output_Point3FI
 * @return 0: distance is 0; 1: ok
 */
int VeloCalib::convLRDI2XYZI(PointLRDI_ptr input_PointLRDI, Point3FI &output_Point3FI)
{
    LaserCorrection corrections = in_param[(int)input_PointLRDI->line_id];
    double distance = (double)input_PointLRDI->distance * DISTANCE_RESOLUTION;
    if(distance==0)
    {
        return 0;
    }
    distance += corrections.dist_correction;

    double cos_vert_angle = corrections.cos_vert_correction;
    double sin_vert_angle = corrections.sin_vert_correction;
    double cos_rot_correction = corrections.cos_rot_correction;
    double sin_rot_correction = corrections.sin_rot_correction;

    double cos_rot_angle =
            cos_rot_table[input_PointLRDI->rot_angle] * cos_rot_correction +
            sin_rot_table[input_PointLRDI->rot_angle] * sin_rot_correction;
    double sin_rot_angle =
            sin_rot_table[input_PointLRDI->rot_angle] * cos_rot_correction -
            cos_rot_table[input_PointLRDI->rot_angle] * sin_rot_correction;

    double horiz_offset = corrections.horiz_offset_correction;
    double vert_offset = corrections.vert_offset_correction;

    // Compute the distance in the xy plane (w/o accounting for rotation)
    /**the new term of 'vert_offset * sin_vert_angle'
                * was added to the expression due to the mathemathical
                * model we used.
                */
    double xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

    // Calculate temporal X, use absolute value.
    double xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    // Calculate temporal Y, use absolute value
    double yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    if (xx < 0) xx=-xx;
    if (yy < 0) yy=-yy;

    // Get 2points calibration values,Linear interpolation to get distance
    // correction for X and Y, that means distance correction use
    // different value at different distance
    double distance_corr_x = 0;
    double distance_corr_y = 0;
    if (corrections.two_pt_correction_available) {
        distance_corr_x =
                (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 240) / (2504 - 240)
                + corrections.dist_correction_x;
        distance_corr_x -= corrections.dist_correction;
        distance_corr_y =
                (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 193) / (2504 - 193)
                + corrections.dist_correction_y;
        distance_corr_y -= corrections.dist_correction;
    }

    double distance_x = distance + distance_corr_x;
    /**the new term of 'vert_offset * sin_vert_angle'
                * was added to the expression due to the mathemathical
                * model we used.
                */
    xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
    ///the expression wiht '-' is proved to be better than the one with '+'
    double x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

    double distance_y = distance + distance_corr_y;
    xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
    /**the new term of 'vert_offset * sin_vert_angle'
                * was added to the expression due to the mathemathical
                * model we used.
                */
    double y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

    // Using distance_y is not symmetric, but the velodyne manual
    // does this.
    /**the new term of 'vert_offset * cos_vert_angle'
                * was added to the expression due to the mathemathical
                * model we used.
                */
    double z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

    /** Use standard ROS coordinate system (right-hand rule: left y,front x) */
//    double x_coord = y;
//    double y_coord = -x;
//    double z_coord = z;
    /** Use our coordinate system (right-hand rule: right x,front y) */
    double x_coord = x;
    double y_coord = y;
    double z_coord = z;

    /** Intensity Calculation */
    double min_intensity = corrections.min_intensity;
    double max_intensity = corrections.max_intensity;

    int intensity1 = (int)input_PointLRDI->intensity;

    double focal_offset = 256
            * (1 - corrections.focal_distance / 13100)
            * (1 - corrections.focal_distance / 13100);
    double focal_slope = corrections.focal_slope;
    int intensity2 = intensity1 + focal_slope *
            (fabs(focal_offset - 256 *
                 (1 - (double)input_PointLRDI->distance/65535)*(1 - (double)input_PointLRDI->distance/65535)));
    if(intensity2 < min_intensity)
    {
        intensity2 = min_intensity;
    }
    if(intensity1>max_intensity)
    {
        intensity2 = max_intensity;
    }

    // convert polar coordinates to Euclidean XYZ
    output_Point3FI.x = x_coord;
    output_Point3FI.y = y_coord;
    output_Point3FI.z = z_coord;
    output_Point3FI.intensity = intensity2;
}

void VeloCalib::createTable()
{
    for(int i=0;i<36000;i++)
    {
        cos_rot_table[i] = cos((double)(i*M_PI/180.0/100.0));
        sin_rot_table[i] = sin((double)(i*M_PI/180.0/100.0));
    }
    for(int i=0;i<LASER_NUM;i++)
    {
        in_param[i].cos_rot_correction = cos(M_PI/180.0*in_param[i].rot_correction);
        in_param[i].cos_vert_correction = cos(M_PI/180.0*in_param[i].vert_correction);
        in_param[i].sin_vert_correction = sin(M_PI/180.0*in_param[i].vert_correction);
        in_param[i].sin_rot_correction = sin(M_PI/180.0*in_param[i].rot_correction);
    }
}

int VeloCalib::readFile(char *file_dir)
{
    tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();
    doc->LoadFile( file_dir );
    int erro_id = doc->ErrorID();
    if(erro_id==tinyxml2::XML_SUCCESS)
    {
        printf("LOG:XML_SUCCESS\n");
    }
    if(erro_id==tinyxml2::XML_ERROR_FILE_NOT_FOUND)
    {
        printf("ERO:XML_ERROR_FILE_NOT_FOUND\n");
        return 0;
    }
    if(erro_id==tinyxml2::XML_ERROR_FILE_COULD_NOT_BE_OPENED)
    {
        printf("ERO:XML_ERROR_FILE_COULD_NOT_BE_OPENED\n");
        return 0;
    }
    if(erro_id==tinyxml2::XML_ERROR_FILE_READ_ERROR)
    {
        printf("ERO:XML_ERROR_FILE_READ_ERROR\n");
        return 0;
    }
    tinyxml2::XMLElement * db = doc->RootElement()->FirstChildElement("DB");
    tinyxml2::XMLElement * min_intensity = db->FirstChildElement("minIntensity_")->FirstChildElement("item");
    tinyxml2::XMLElement * max_intensity = db->FirstChildElement("maxIntensity_")->FirstChildElement("item");
    tinyxml2::XMLElement * points = db->FirstChildElement("points_")->FirstChildElement("item");
    for(int i=0;i<LASER_NUM;i++)
    {
        printf("laser:%d \n",i);
        in_param[i].min_intensity = min_intensity->IntText();
        printf("min_i:%03d ",in_param[i].min_intensity);
        in_param[i].max_intensity = max_intensity->IntText();
        printf("max_i:%03d ",in_param[i].max_intensity);
        in_param[i].rot_correction = points->FirstChild()->
                FirstChildElement("rotCorrection_")->DoubleText();
        printf("rot_corr:%.3f ",in_param[i].rot_correction);
        in_param[i].vert_correction = points->FirstChild()->
                FirstChildElement("vertCorrection_")->DoubleText();
        printf("vert_corr:%.3f ",in_param[i].vert_correction);
        in_param[i].dist_correction = points->FirstChild()->
                FirstChildElement("distCorrection_")->DoubleText();
        printf("dist_corr:%.3f ",in_param[i].dist_correction);
        in_param[i].dist_correction_x = points->FirstChild()->
                FirstChildElement("distCorrectionX_")->DoubleText();
        printf("dist_corr_x:%.3f ",in_param[i].dist_correction_x);
        in_param[i].dist_correction_y = points->FirstChild()->
                FirstChildElement("distCorrectionY_")->DoubleText();
        printf("dist_corr_y:%.3f ",in_param[i].dist_correction_y);
        in_param[i].vert_offset_correction = points->FirstChild()->
                FirstChildElement("vertOffsetCorrection_")->DoubleText();
        printf("vert_offset:%.3f ",in_param[i].vert_offset_correction);
        in_param[i].horiz_offset_correction = points->FirstChild()->
                FirstChildElement("horizOffsetCorrection_")->DoubleText();
        printf("horiz_offset:%.3f ",in_param[i].horiz_offset_correction);
        in_param[i].focal_distance = points->FirstChild()->
                FirstChildElement("focalDistance_")->DoubleText();
        printf("focal_dist:%.3f ",in_param[i].focal_distance);
        in_param[i].focal_slope = points->FirstChild()->
                FirstChildElement("focalSlope_")->DoubleText();
        printf("focal_slope:%.3f\n",in_param[i].focal_slope);

        in_param[i].two_pt_correction_available = false;
        scan_table[i].x = i;
        scan_table[i].y = in_param[i].vert_correction;

        min_intensity = min_intensity->NextSiblingElement();
        max_intensity = max_intensity->NextSiblingElement();
        points = points->NextSiblingElement();
    }
    delete doc;
    return 1;
}

void VeloCalib::quickSort(Point2F *s, int l, int r)
{
    if (l < r)
    {
        int i = l, j = r;
        Point2F x = s[l];
        while (i < j)
        {
            while(i < j && s[j].y >= x.y)
                j--;
            if(i < j)
                s[i++] = s[j];
            while(i < j && s[i].y < x.y)
                i++;
            if(i < j)
                s[j--] = s[i];
        }
        s[i] = x;
        quickSort(s, l, i - 1);
        quickSort(s, i + 1, r);
    }
}



