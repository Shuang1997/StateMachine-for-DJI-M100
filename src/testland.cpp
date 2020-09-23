#include "Functions.h"
#include <iostream>



int main(int argc,char** argv)
{
  ros::init(argc,argv,"testlanding");
  ros::NodeHandle n;
  DJIDrone* drone = new DJIDrone(n);
  ros::Subscriber delt = n.subscribe<guidance::distance>("guidance/relative",1000,landingCallback);
  ros::Subscriber local_position = n.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, local_position_callback);
  ros::Subscriber quaternion = n.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 10, AttitudeQuaternion_callback);
  ros::Subscriber flight_status = n.subscribe<std_msgs::UInt8>("dji_sdk/flight_status",10,flight_status_callback);  
  drone->request_sdk_permission_control();
  int MissionStart = Takeoff(drone);
  FlyToTargetPoint(drone,n,4,-5,3,0);
  sleep(5);
  cout<<"I am backing!"<<endl;
  //FlyToTargetPoint(drone,n,0,0,2.5,0);
  Landing(n,drone);
  drone->release_sdk_permission_control();
  return 0;
}


// void Landing(ros::NodeHandle n, DJIDrone* drone)
// {
//   FlyToTargetPoint(drone,n,0,0,2.5,0);
//   while(1)   
//   {
// //drone->landing();
// ros::spinOnce();
//     
//       
//     if(delt.dx == 160 && delt.dy == 120)
//     {
//       while(1)
//       {
//         ros::spinOnce();
// 	drone->velocity_control(0,0,0,0,0);
// 	if(delt.dx != 160 || delt.dy != 120)
// 	  break;
//       }      
//     }
//     else
//     {
//       while(1)
//       {
// 	ros::spinOnce();
// 	drone->velocity_control(0,0.5*delt.dy/abs(delt.dy),-0.5*delt.dx/abs(delt.dx),0,0);
// 	if((abs(delt.dx)*abs(delt.dx)+abs(delt.dy)*abs(delt.dx))< 400 || (delt.dx == 160 && delt.dy == 120))
// 	{
// 	  if((abs(delt.dx)*abs(delt.dx)+abs(delt.dy)*abs(delt.dx))< 400)
// 	    goto here;
// 	  else
// 	    break;
// 	}
//       }  
//     }
//   } 
// here:drone->landing();
// }