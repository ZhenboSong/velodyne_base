#include "velo_calib.h"
#include <iostream>
#include <time.h>
using namespace std;

int main()
{
    char dir[128] = "../data/data.xml";
    VeloCalib test(dir);
    PointLRDI_ptr point = new PointLRDI;
    Point3FI out;
    point->distance = 800;
    point->intensity = 50;
    point->line_id = 31;
    point->rot_angle = 0;
    clock_t loadTime = clock();
    test.convLRDI2XYZI(point,out);
    clock_t deleteTime = clock();
    cout<<test.getTopXLaser(4)<<"  "<<test.getGapXYLaser(40,50)<<endl;
    printf( "conv time=%u\n", (unsigned)(deleteTime - loadTime) );
    cout <<"("<<out.x<<","<<out.y<<","<<out.z<<","<<(int)out.intensity<<")"<<endl;
    return 0;
}
