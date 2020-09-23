// // HIT Algricutural
// // ISAP All Right Reserved 2017.10
// 
#include "Functions.h"   
#include "SimpleGPIO.h"
dji_sdk::LocalPosition local;
dji_sdk::AttitudeQuaternion q;
std_msgs::UInt8 myflight_status;
 
double yaw_R;
double yaw0_D;
double yaw0_R;
float32_t X, Y, yawT_R;
double X0;
double Y0;
double Z0;
unsigned int LEDGPIO = 158;
// target number position  10 numbers in total
double TargetNumber_Position[10][4];  // 0 - 10
double Number_Position[10][3];
double FigureSum[10][3];
double Screen_Position[3];
double Screen_Yaw;

double Height;
double DistanceToWall;

double yaw;
double pitch;
double roll;

double VtoWall;

double Target_X;
double Target_Y;
double Target_Z;

float32_t VX,VY;

double StartTime;
double MissionDuration;

int Figure;

double d_yaw;

double Height_HigherPoint;
double Height_LowerPoint;
double d_Height;

// 
// // mission functionsTargetFigure
// 
// void ParameterInput() // global  
// {
//   
// }
// 
// 
int Takeoff(DJIDrone* drone)
{
  gpio_set_value(LEDGPIO, HIGH);
  drone->activate();
  sleep(1);
  cout<<"Preparing for Taking off"<<endl;
  ros::Rate takeoff(1.5);
  while(ros::ok()) // keep waiting
  {
    ros::spinOnce();
    cout<<"Taking off state:  "<<myflight_status.data<<endl;
    if( myflight_status.data == 3  ) // Onboard SDK
    {
        StartTime =ros::Time::now().toSec();
	printf(" Taking off Successed !  \n");
        break;
    }
    takeoff.sleep();
  }
  //求初始偏航
  sleep(0.2); 
  
  int i = 0;
  float qq0,qq1,qq2,qq3;
  double yaw_sum = 0;
  double x_sum = 0;
  double y_sum = 0;
  double z_sum = 0;
  
  ros::Rate rate(20);
  while(i < 60 && ros::ok())  //3秒时间偏航角均值
  {
    ros::spinOnce();
    qq0 = drone->attitude_quaternion.q0;
    qq1 = drone->attitude_quaternion.q1;
    qq2 = drone->attitude_quaternion.q2;
    qq3 = drone->attitude_quaternion.q3;
    Quaternion_To_Euler(qq0,qq1,qq2,qq3);
    yaw_sum = yaw_sum + yaw;
    
    x_sum = x_sum + local.x;
    y_sum = y_sum + local.y;
    z_sum = z_sum + local.z;
   
    i = i + 1;
    rate.sleep();
  }
  
  yaw0_D = yaw_sum / 60;
  yaw0_R = yaw0_D * D2R;  //起飞系相对于正北系偏航
  
  X0 = x_sum / 60;
  Y0 = y_sum / 60;
  Z0 = z_sum / 60;
 
  cout<<"yaw0 = "<<yaw0_D<<" deg "<<endl;
  cout<<"X0 = "<<X0<<" m "<<endl;
  cout<<"Y0 = "<<Y0<<" m "<<endl;
  cout<<"Z0 = "<<Z0<<" m "<<endl;
  
  X0 = 0;
  Y0 = 0;
  Z0 = 0;
  
  return 1;
}

void Initialization()
{
  int i = 0;
  while(i < 10)
  {
    Number_Position[i][0] = -1000;
    Number_Position[i][1] = -1000;
    Number_Position[i][2] = -1000;
    FigureSum[i][0] = 0;
    FigureSum[i][1] = 0;
    FigureSum[i][2] = 0;
    i = i + 1;
  }
  
  i = 0;
  ros::Rate rate(5);
  while(ros::ok() && i < 2)
  {
    ros::spinOnce();
    i = i + 1;
    rate.sleep();
  }

  i = 0;
  int j;
  
  while(ros::ok() && i < 15)
  {
    ros::spinOnce();
    j = 0;
    while(ros::ok() && j < 10) // Figure = j
    {
      FigureSum[j][0] = FigureSum[j][0] + Number_Position[j][0]; //+ local.x;
      FigureSum[j][1] = FigureSum[j][1] + Number_Position[j][1]; //+ local.y;
      FigureSum[j][2] = FigureSum[j][2] + Number_Position[j][2];// + local.z;
      if(j == 6){
      cout<<Number_Position[j][0]<<endl;
      cout<<FigureSum[j][0]<<endl<<endl;}
      j = j + 1;
    }
    i = i + 1;
    
    rate.sleep();
  }
  j = 0;
  while(ros::ok() && j < 10) // Figure = j
  {
    TargetNumber_Position[j][0] = j;
    TargetNumber_Position[j][1] = FigureSum[j][0] / 15;
    TargetNumber_Position[j][2] = FigureSum[j][1] / 15;
    TargetNumber_Position[j][3] = FigureSum[j][2] / 15;
    if(j == 6){
      cout<<TargetNumber_Position[j][1]<<endl;
      cout<<TargetNumber_Position[j][2]<<endl;
      cout<<TargetNumber_Position[j][3]<<endl<<endl;}
    j = j + 1;
  }
  
  cout<<"Initialization Finished ! "<<endl<<endl;
}
// 
// 
int TargetFigureRecognization()
{
//   TargetNumber_Position[5][0] = 6;
//   TargetNumber_Position[5][1] = 1;
//   TargetNumber_Position[5][2] = 1;
//   TargetNumber_Position[5][3] = 1.5;
  
  Figure = 6;
  return Figure;
}

void SprayMission(DJIDrone* drone, ros::NodeHandle n, int TargetFigure)
{
  int Continue;
  int i = 1;
  while(i <= 10)
  {
    if( TargetNumber_Position[i][0] == TargetFigure )
    {
      Target_X = TargetNumber_Position[i][1] - 1.1;
      Target_Y = TargetNumber_Position[i][2];
      Target_Z = TargetNumber_Position[i][3] + 0.05;
      break;
    }
    i = i + 1;
  }
  
  cout<<"Target Figure = "<<TargetFigure<<endl;
  cout<<"Target X = "<<Target_X<<endl;
  cout<<"Target Y = "<<Target_Y<<endl;
  cout<<"Target Z = "<<Target_Z<<endl;
  
  cout<<"Continue ?"<<endl;
  scanf("%d",&Continue);
  
  if(Continue == 0)
  {
    ros::Rate Alert(1);
    while(ros::ok())
    {
      cout<<"Red Alert ! Switch to Pilot ! "<<endl;
      Alert.sleep();
    }
  }
    
  FlyToTargetPoint(drone,n,Target_X,Target_Y,Target_Z,0);  //float inputx,float inputy,float inputz,float yawT_D 
  
//   ros::Rate Aim(2);
//   while(ros::ok() && i < 2)
//   {
//     ros::spinOnce();
//     i = i + 1;
//     Aim.sleep();
//   }
//   cout<<"Move right = "<<Number_Position[TargetFigure][1]<<endl;
//   cout<<"Move up = "<<Number_Position[TargetFigure][2]<<endl<<endl;
//   FlyToTargetPoint(drone,n,Target_X,Target_Y + Number_Position[TargetFigure][1],Target_Z + Number_Position[TargetFigure][2],0);  //float inputx,float inputy,float inputz,float yawT_D 
//   cout<<"Aim Finished !"<<endl;
  
  VtoWall = 0.05;
  ros::Rate close(20);
  while(ros::ok())
  {
    ros::spinOnce();
    MoveAtCertainVelocity(drone, n, VtoWall, 0, 0, 0);
    cout<<"DistanceToWall = "<<DistanceToWall<<endl;
    if(DistanceToWall < 0.7  && DistanceToWall > 0.5 )
    {
      MoveAtCertainVelocity(drone, n, 0, 0, 0, 0);
      // water on
      cout<<"Water on !"<<endl;
      gpio_set_value(LEDGPIO, LOW);
      sleep(3);
      // water off
      gpio_set_value(LEDGPIO, HIGH);
      cout<<"Water off !"<<endl;
      // descend
      //ros::spinOnce();
      //Height_HigherPoint = Height;
      //while(1)
      //{
// 	MoveAtCertainVelocity(drone, n, 0, 0, -0.1, 0);
// 	ros::spinOnce();
// 	Height_LowerPoint = Height;
// 	d_Height = Height_HigherPoint - Height_HigherPoint;
// 	if(d_Height > 0.15)
// 	  break;
//       }  
      // stop descending
      sleep(3);
      MoveAtCertainVelocity(drone, n, 0, 0, 0, 0);
      //  water on
      gpio_set_value(LEDGPIO, LOW);
      cout<<"Water on !"<<endl;
      sleep(3);
      // water off
      gpio_set_value(LEDGPIO, HIGH);
      cout<<"Water off !"<<endl;
      cout<<"Spray Finished ! "<<endl;
      break;
    }
    close.sleep();
  }
}


void Landing(ros::NodeHandle n, DJIDrone* drone)
{
  FlyToTargetPoint(drone,n,X0,Y0,Z0,0);
  while(1)
  {
    MoveAtCertainVelocity(drone, n, 0, 0, -0.1, 0);
    ros::spinOnce();
    if(Height < 0.25)
      break;
  } 
  sleep(1);
  drone->landing();
}


// basic functions

int Timer()
{
    MissionDuration = ros::Time::now().toSec() - StartTime;
    
    if( (MissionDuration < 50 && MissionDuration > 48) ||  (MissionDuration < 110 && MissionDuration > 109) || (MissionDuration < 170 && MissionDuration > 169) || (MissionDuration < 230 && MissionDuration > 228)  )
    {
        return 1;
    } //single mission time out
    
    // 600 s  in  total
    if( (ros::Time::now().toSec() - StartTime) > 400 )
    {
        return 2; // total time out
    }
    
    return 0;
}


void FlyToTargetPoint(DJIDrone* drone,ros::NodeHandle a,float inputx,float inputy,float inputz,float yawT_D) 
{//输入起飞坐标系下坐标，输入yaw为目标状态相对于起飞坐标系的偏航角，利用GPS导航信息
  //坐标变换：起飞系->API参考系
  
  cout<<"X (forward) is:"<<inputx<<endl;
  cout<<"Y (right) is:"<<inputy<<endl;
  cout<<"Yaw (relative to forward) is:"<<yawT_D<<endl<<endl;
  
  // yaw0_R 起飞系相对于正北系偏航
  yawT_R = yawT_D / 180 * pi;  // 目标状态相对于起飞坐标系的偏航
  X = inputx*cos(yaw0_R)-inputy*sin(yaw0_R) + X0;
  Y = inputx*sin(yaw0_R)+inputy*cos(yaw0_R) + Y0;

  cout<<"X (north) is:"<<X<<endl;
  cout<<"Y (east) is:"<<Y<<endl;
  cout<<"Target yaw (relative to north) is: "<<yawT_D<<" deg"<<endl;
  sleep(1);
  cout<<"Move Move Move ! "<<endl;
  
  while(ros::ok())
  {
    drone->local_position_control(X, Y, inputz, yaw0_R + yawT_R);  //零点是什么？
    ros::spinOnce();
/*
    cout<<"local.x is:"<<local.x<<endl;
    cout<<"local.y is:"<<local.y<<endl;
    cout<<"local.z is:"<<local.z<<endl;
    cout<<"Psic is:"<<psi<<endl;
    */
    d_yaw = abs(atan2(2.0f*(q.q0*q.q3+q.q1*q.q2), 1-2.0f*(q.q3*q.q3+q.q2*q.q2))*R2D - (yaw0_R+yawT_R) * R2D);
    if( abs(local.x - X)<0.5 & abs(local.y - Y)<0.5 &  d_yaw< 2)  
    {
      break;
    }
  }
  cout<<"Arrived at Target Point ! "<<endl;
}

void MoveAtCertainVelocity(DJIDrone* drone,ros::NodeHandle a,float inputvx,float inputvy,float inputvz,float inputvyaw) //起飞坐标系，GPS
{
  VX = inputvx*cos(yaw0_R)-inputvy*sin(yaw0_R);
  VY = inputvx*sin(yaw0_R)+inputvy*cos(yaw0_R);
  drone->velocity_control(0,VX,VY,inputvz,inputvyaw);
}


void Quaternion_To_Euler(float q0,float q1,float q2,float q3)
{
  float r11,r12,r21,r31,r32,r1,r2,r3;
  r11 = 2.0f *(q1 * q2 + q0 * q3);
  r12 = q0 * q0 + q1 * q1 - q2 * q2  - q3 * q3 ;
  r21 = -2.0f * (q1 * q3 - q0 * q2);
  r31 = 2.0f *( q2 * q3 + q0  * q1);
  r32 = q0 * q0 - q1 * q1  - q2 * q2 + q3 * q3 ;
  
  yaw = atan2( r11, r12 );
  pitch = asin( r21 );
  roll = atan2( r31, r32 );
  
  yaw=yaw*180.0/3.1415926; // -180 ~ +180 deg
  pitch=pitch*180.0/3.1415926;  // deg
  roll=roll*180.0/3.1415926;  // deg
}


// Call Back

void messageCallback(const mono_pub::MonoMsg::ConstPtr& msg)  //Mono position
{
  vector<mono_pub::Num>   mono_status = msg->mono_status;  
  for (int i = 0; i < mono_status.size(); i++)  
    {  
        mono_pub::Num num = mono_status.at(i);   
        // ROS_INFO("I heard num: [%d] ", num.id);
	// ROS_INFO("X cordinate of num: [%d] is [%lf]", num.id,-num.position.z);
	// ROS_INFO("Y cordinate of num: [%d] is [%lf]", num.id,num.position.x);
	// ROS_INFO("Z cordinate of num: [%d] is [%lf]", num.id,num.position.y);
	Number_Position[num.id][0] = -num.position.z;
	Number_Position[num.id][1] = num.position.x;
	Number_Position[num.id][2] = num.position.y;
	// 
    }  

//   	     num.id=info.number;
//    	     num.position=position;
//    	     num.attitude=attitude;
//	     position.x = info.x;
//  	     position.y = info.y;
//   	     position.z = info.z;
//   	     attitude.yaw = info.yaw;
//   	     attitude.pitch = info.pitch;
//   	     attitude.roll = info.roll;
//
}

void ultrasonic_callback(sensor_msgs::LaserScan g_ul)  //Guidance SDK
{

  Height = g_ul.ranges[1];
  DistanceToWall = g_ul.ranges[0];
  //cout<<"Height = "<<Height<<endl;
  cout<<"DistanceToWall = "<<DistanceToWall<<endl;
}

void local_position_callback(const dji_sdk::LocalPosition::ConstPtr& msg)  //onboard sdk  GPS
{
  local.x = msg->x;
  local.y = msg->y;
  local.z = msg->z;
  local.header = msg->header;
  local.ts = msg->ts;
  // ROS_INFO("local_position_z is: [%f]",msg->z);
}

void AttitudeQuaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr& Q)   //onboard sdk  GPS
{ 
  q.q0 = Q->q0;
  q.q1 = Q->q1;
  q.q2 = Q->q2;
  q.q3 = Q->q3;
  yaw_R = atan2(2.0f*(q.q0*q.q3+q.q1*q.q2),1-2*(q.q3*q.q3+q.q2*q.q2));
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)  //onboard sdk  GPS
{
  myflight_status.data = msg->data;
}

