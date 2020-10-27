/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */


#include "flight_control_interface.h"
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/*! main
 *
 */
int flight_control_order(vector<Rect1> multi_result,Vehicle*   vehicle)
{
   	float vx=0.0;
    float vy=0.0;
    float vz=0.0;
	int tar_num = multi_result.size();
    if(tar_num==2){
        Rect1 tar1;
        Rect1 tar2;
        bool flag= false;
	    if((multi_result[0].x>multi_result[1].x)&&(multi_result[0].z<multi_result[1].z)){
                tar1 = multi_result[1];
                tar2 = multi_result[0];
                flag= true;
        }
        if((multi_result[0].x<multi_result[1].x)&&(multi_result[0].z>multi_result[1].z)){
                tar1 = multi_result[0];
                tar2 = multi_result[1];
                flag= true;
        }
        if(flag){
                float theta1 = 105.0;
                float theta2 = 75.0;
                float k1 = tan(theta1*M_PI/180.0);
                float k2 = tan(theta2*M_PI/180.0);
		//float dest_x = tan(theta1*M_PI/180.0) * dest_x - tan(theta1*M_PI/180.0) * tar1.x + tar2.z;
		//float dest_z = tan(theta2*M_PI/180.0) * dest_x - tan(theta2*M_PI/180.0) * tar2.x + tar2.z;

                float dest_x = (k1*tar1.x - k2*tar2.x +tar2.z-tar1.z)/(k1-k2);
                float dest_z = k1 * dest_x + tar1.z - k1* tar1.x;

                int vy_time_require_ms = 0;
                int vx_time_require_ms = 0;

                int maxtime_ms = 3000;
                int mintime_ms = 1000;
                
                cout<<"tar1:  "<<tar1.x<<"  "<<tar1.y<<"  "<<tar1.z<<endl;
                cout<<"tar2:  "<<tar2.x<<"  "<<tar2.y<<"  "<<tar2.z<<endl;
                cout<<"dest:  "<<dest_x<<"  "<<dest_z<<endl;

                bool move_forward = (((fabs(dest_z) / 1000.0)>=1)&&((fabs(dest_z) / 1000.0)<=10));
                bool move_right = (((fabs(dest_x) / 1000.0)>=1)&&((fabs(dest_x) / 1000.0)<=10));

                if(move_right){

                    vy_time_require_ms = (int)(fabs(dest_x));
                    vy_time_require_ms = (vy_time_require_ms>=maxtime_ms)?maxtime_ms:((vy_time_require_ms<=mintime_ms)?mintime_ms:vy_time_require_ms);
                    vy = (dest_x>0) ? 1.0:-1.0;
                    cout<<"start a left-right flight:  "<<vy<<"  "<<vy_time_require_ms<<endl;
                    moveByPositionOffset(vehicle, 0,vy,0.0,0.0,vy_time_require_ms);

                }
                if(move_forward){

                    vx_time_require_ms = (int)(fabs(dest_z));
                    vx_time_require_ms = (vx_time_require_ms>=maxtime_ms)?maxtime_ms:((vx_time_require_ms<=mintime_ms)?mintime_ms:vx_time_require_ms);
                    vx = (dest_z>0) ? 1.0:-1.0;
                    cout<<"start a fore-back flight:  "<<vx<<"  "<<vx_time_require_ms<<endl;
                    moveByPositionOffset(vehicle, vx,0,0.0,0.0,vx_time_require_ms);

                }

        }
    }



	if(tar_num==1){
		    float vx=0.0;
		    float vy=0.0;
		    float vz=0.0;
		    int vx_time_require_ms = 0;
		    int vy_time_require_ms = 0;
		    int z_dis = 4000;
		    bool move_forward = (((multi_result[0].z - z_dis) / 1000.0)>=1)&&(((multi_result[0].z - z_dis) / 1000.0)<=10);
            bool move_right = (abs(multi_result[0].x/1000.0)<=10)&&(abs(multi_result[0].x/1000.0)>=1);
            if(move_right && (!move_forward)){
                    cout<<"start a new flight control"<<endl;
                    int maxtime_ms = 3000;
                                    
                    vy_time_require_ms = (int)((abs(multi_result[0].x / 1000.0) / (1.0))*1000);
                    if(vy_time_require_ms>=maxtime_ms){
                        vy_time_require_ms = maxtime_ms;
                    }
                    if(vy_time_require_ms<=2000){
                        vy_time_require_ms = 2000;
                    }
                    cout<<"dis to center: "<< (multi_result[0].x / 1000.0)<<"  "<<vy_time_require_ms<<endl;
                    if((multi_result[0].x / 1000.0)>0){
                        vy = 1.0;
                    }
                    if((multi_result[0].x / 1000.0)<0){
                        vy = -1.0;
                    }
                    cout<<"rec vy  "<<  vy<<endl;
                    moveByPositionOffset(vehicle, 0,vy,0.0,0.0,vy_time_require_ms);
                    cout<<"finish a new flight control in vy "<<endl;
             }
            if(move_forward && (!move_right)){
                                vx_time_require_ms = (int)((fabs(((multi_result[0].z - z_dis) / 1000.0)) / (1.0))*1000);
                                if(vx_time_require_ms>=2000){
                                    vx_time_require_ms = 2000;
                                }
                                if(vx_time_require_ms<=1000){
                                    vx_time_require_ms = 1000;
                                }
                                cout<<"dis to 3m: "<< ((multi_result[0].z - z_dis) / 1000.0)<<"  "<<vx_time_require_ms<<endl;
                                if(((multi_result[0].z - z_dis) / 1000.0)>0){
                                    vx = 1.0;
                                }
                                if(((multi_result[0].z - z_dis) / 1000.0)<0){
                                    vx = -1.0;
                                }
                                cout<<"rec vx  "<<  vx<<endl;

                                moveByPositionOffset(vehicle, vx,0,0.0,0.0,vx_time_require_ms);

                                cout<<"finish a new flight control in vx "<<endl;
            }
                       
                        
            if(move_forward && move_right)       
            {
                                cout<<"dis to center: "<< (multi_result[0].x / 1000.0)<<"  "<<vy_time_require_ms<<endl;
                                if((multi_result[0].x / 1000.0)>0){
                                    vy = 1.0;
                                }
                                if((multi_result[0].x / 1000.0)<0){
                                    vy = -1.0;
                                }
                                cout<<"rec vy  "<<  vy<<endl;

                                cout<<"dis to 3m: "<< ((multi_result[0].z - z_dis) / 1000.0)<<"  "<<vx_time_require_ms<<endl;
                                if(((multi_result[0].z - z_dis) / 1000.0)>0){
                                    vx = 1.0;
                                }
                                if(((multi_result[0].z - z_dis) / 1000.0)<0){
                                    vx = -1.0;
                                }
                                cout<<"rec vx  "<<  vx<<endl;

                                moveByPositionOffset(vehicle, vx,vy,0.0,0.0,1000);

                                cout<<"finish a new flight control in vx and vy"<<endl;
            }
 

    }
     
    return 0;
}
