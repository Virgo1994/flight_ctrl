//
// Created by lining on 7/8/20.
//

#ifndef ONBOARDSDK_FLIGHT_CONTROL_INTERFACE_H
#define ONBOARDSDK_FLIGHT_CONTROL_INTERFACE_H


#include "flight_control_sample.hpp"
#include <vector>
using namespace std;

struct Rect1
{
    int miny;
    int minx;
    int maxy;
    int maxx;
    float x;
    float y;
    float z;
    float x_ori;
    float y_ori;
    float z_ori;
    int stat;
    int zed;
    float score;
};
int flight_control_order(vector<Rect1> multi_result,Vehicle*   vehicle);

#endif //ONBOARDSDK_FLIGHT_CONTROL_INTERFACE_H
