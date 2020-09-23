// // HIT Algricutural
// // ISAP All Right Reserved 2017.10
// 
#include "Functions.h"   
#include "SimpleGPIO.h"
#include "../../Guidance-SDK-ROS/include/DJI_guidance.h"
dji_sdk::LocalPosition local;
dji_sdk::AttitudeQuaternion q;
std_msgs::UInt8 myflight_status;
guidance::distance delt;
 
double yaw_R;
double yaw0_D;
double yaw0_R;
float32_t X, Y, yawT_R;
double X0;
double Y0;
double Z0;
double X0B;
double Y0B;
unsigned int LEDGPIO = 158;
// target number position  10 numbers in total
double TargetNumber_Position[10][4];  // 0 - 10

double Number_Position[10][4];//[detected][x][y][z]
double bigscreenNumber_Position[10][4];
//int Number_detected;//judge the number is detected
int detectednumbers = 0;
int detected_all_numbers = 3;//1 is represented all numbers has been detected
int initializationtime = 45;

int C[3] = {4,-10,2};//the third fixed point C also the digital screen's position     

double FigureSum[10][3];
double CloseFigureSum[3];
double AccurateFigurePosi[3];
double Screen_Position[3];
double Screen_Yaw;

double Height;
double DistanceToWall;

double yaw;
double pitch;
double roll;

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
double Body_x;
double Body_y;
double d_X;
double d_Y;
double d_Z;
double vx;
double vy;
double vz;
double va;

int OneFigureInSight;
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
  sleep(2);
  cout<<"Preparing for Taking off"<<endl;
  int i = 0;
  float qq0,qq1,qq2,qq3;
  double yaw_sum = 0;
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
    
    i = i + 1;
    rate.sleep();
    // cout<<local.x<<endl;
  }
  yaw0_D = yaw_sum / 60;
  yaw0_R = yaw0_D * D2R;  //起飞系相对于正北系偏航
  cout<<"Yaw_0 = "<<yaw0_D<<"deg"<<endl;
  
  ros::Rate takeoff(1);
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
  
  sleep(7); 
  return 1;
}

void Initialization(DJIDrone* drone, ros::NodeHandle n)
{
  time_t start_time;
  time_t end_time;
  start_time = time(NULL);
  //single mission time now single mission timer 
  //fly to the first fixed point A which is in the right to the big screen
  int A[3] = {4,1,2};
  FlyToTargetPoint(drone,n,A[0],A[1],A[2],0);
  cout<< "arrived at A" << "\n" << endl;
   
  int i = 0;
//  initial the numbers which will be watched in the future
  while(i < 10)
  {
    Number_Position[i][0] = 0;//1 is detected, 0 is not detected
    Number_Position[i][1] = -1000;//initial the position of numbers
    Number_Position[i][2] = -1000;
    Number_Position[i][3] = -1000;  
    FigureSum[i][0] = 0;//the position of small figures relative to word coordination
    FigureSum[i][1] = 0;
    FigureSum[i][2] = 0;  
    i = i + 1;
  }

  //fly to the second fixed point B which is in the left to the big screen 
  ros::Rate rate(5);
  int velocity_judge = 0.6; //
  int distance_judge = 10;
  int count = 0;
  end_time = time(NULL);
  while((end_time-start_time) <= initializationtime)//if all numbers have not been detected or the time is sufficient, the drone will fly from left to right and come back
  {
    int j;
    while(ros::ok())
      {
	ros::spinOnce();
	
	//keeping X Z and yaw
	float X,Y,Z;
	X = local.x*cos(yaw0_R) + local.y*sin(yaw0_R);
	Y = local.y*cos(yaw0_R) - local.x*sin(yaw0_R);
	Z = local.z;
	d_yaw = (yaw0_R+yawT_R)*R2D - atan2(2.0f*(q.q0*q.q3+q.q1*q.q2), 1-2.0f*(q.q3*q.q3+q.q2*q.q2))*R2D;
    
	drone->velocity_control(0,(A[0]-X)*KX,-velocity_judge,(A[2]-Z)*KZ,KA*d_yaw);
	
	j = 0;
	while(ros::ok() && j < 10)
	{
	  //if(Number_Position[j][1] > 0.1 && j == Number_detected)
	  if(Number_Position[j][1] > 0.1)
	  {
	    //judge the number whether has been watched
	    if(Number_Position[j][0] == 0)//not watched->watched
	    {
	      i = 0;
	      while(ros::ok() && i < 5)
	      {
		FigureSum[j][0] = FigureSum[j][0] + Number_Position[j][1] + local.x*cos(yaw0_R) + local.y*sin(yaw0_R);
		FigureSum[j][1] = FigureSum[j][1] + Number_Position[j][2] + local.y*cos(yaw0_R) - local.x*sin(yaw0_R);
		FigureSum[j][2] = FigureSum[j][2] + Number_Position[j][3] + local.z;
		i = i + 1;
	      }
	      TargetNumber_Position[j][0] = j;
	      TargetNumber_Position[j][1] = FigureSum[j][0] / i;
	      TargetNumber_Position[j][2] = FigureSum[j][1] / i;
	      TargetNumber_Position[j][3] = FigureSum[j][2] / i;
	      cout<< j <<" has been watched"<< "\n" << endl;
	      cout<<TargetNumber_Position[j][1]<<endl;
	      cout<<TargetNumber_Position[j][2]<<endl;
	      cout<<TargetNumber_Position[j][3]<<endl;
	      detectednumbers = detectednumbers + 1;
	      Number_Position[j][0] = 1;
	      cout<< detectednumbers << " numbers have been watched" << "\n" << endl;
	      if(detectednumbers == detected_all_numbers)
	      {
		//cout<< "detecting all numbers is successful!" << "\n" << endl;
		break;
	      }
	      end_time = time(NULL);
	      //cout<< (end_time-start_time) << "\n" << endl;
	      if( (end_time-start_time) >= initializationtime )
	      {
		//cout<< "detecting is over time : " << initializationtime << "\n" << endl;
		break;
	      }
	    }
	  }
	  j = j + 1;
	  end_time = time(NULL);
	  //cout<< (end_time-start_time) << "\n" << endl;
	  if( (end_time-start_time) >= initializationtime )
	  {
	    //cout<< "detecting is over time : " << initializationtime << "\n" << endl;
	    break;
	  }
	  //rate.sleep();
	}
	if(count%2 ==0 && Y < -distance_judge)//10 is the distance between A and B
	{
	  cout<< "arrived at B" << "\n" << endl;velocity_judge = -velocity_judge;
	  break;
	}
	if(count%2 !=0 && Y > 0)//10 is the distance between A and B
	{  
	  cout<< "arrived at A" << "\n" << endl;velocity_judge = -velocity_judge;
	  cout<< "\n" << endl;
	  break;
	}
	if(detectednumbers == detected_all_numbers)
	{
	  //cout<< "detecting all numbers is successful!" << "\n" <<endl;
	  break;
	}
	end_time = time(NULL);
	//cout<< (end_time-start_time) << "\n" << endl;
	if( (end_time-start_time) >= initializationtime )
	{
	  //cout<< "detecting is over time : " << initializationtime << "\n" << endl;
	  break;
	}
      }    
      count = count + 1;
      if(detectednumbers == detected_all_numbers)
      {
	cout<< " All numbers were successfully detected! " << endl;
	break;
      }
      end_time = time(NULL);
    //cout<< (end_time-start_time) <<endl;
    if( (end_time-start_time) >= initializationtime )
    {
      cout<< "Detecting is over time : " << initializationtime << "\n" << endl;
      break;
    }
  }
  
  //int C[3] = {4,-10,2};//the third fixed point C also the digital screen's position      
  //fly to digital screen
  // FlyToTargetPoint(drone,n,C[0],C[1],C[2],0);
  
  /*
  int j;
  while(ros::ok)
  {
    ros::spinOnce();
    drone->velocity_control(0,0,-1,0,0);
    float X,Y;
    X = local.x*cos(yaw0_R) + local.y*sin(yaw0_R);
    Y = local.y*cos(yaw0_R) - local.x*sin(yaw0_R); 
    j = 0;
    while(ros::ok() && j < 10)
    {
      if(Number_Position[j][1] > 0.1 && j == Number_detected)
      {
	//judge the number whether has been watched
	if(Number_Position[j][0] == 0)//not watched->watched
	{
	  i = 0;
	  while(ros::ok() && i < 15)
	  {
	    FigureSum[j][0] = FigureSum[j][0] + Number_Position[j][1] + local.x*cos(yaw0_R) + local.y*sin(yaw0_R);
	    FigureSum[j][1] = FigureSum[j][1] + Number_Position[j][2] + local.y*cos(yaw0_R) - local.x*sin(yaw0_R);
	    FigureSum[j][2] = FigureSum[j][2] + Number_Position[j][3] + local.z;
	    i = i + 1;
	  }
	  TargetNumber_Position[j][0] = j;
	  TargetNumber_Position[j][1] = FigureSum[j][0] / i;
	  TargetNumber_Position[j][2] = FigureSum[j][1] / i;
	  TargetNumber_Position[j][3] = FigureSum[j][2] / i;
	  cout<< j <<" has been watched"<<endl;
	  cout<<TargetNumber_Position[j][1]<<endl;
	  cout<<TargetNumber_Position[j][2]<<endl;
	  cout<<TargetNumber_Position[j][3]<<endl;
	  detectednumbers = detectednumbers + 1;
	  Number_Position[j][0] = 1;
	  cout<< detectednumbers << " numbers have been watched"<<endl;
	  cout<<"\n"<<endl;
	}
      }
      j = j + 1;
      //rate.sleep();
    }
    if(Y < -10)//10 is the distance between A and B
      break;
  }
  
  cout<<"arrived at B"<<endl;
  */
  /*
  if(end >= 45)
  {
    int C[3] = {4,12,2};//the third fixed point C also the digital screen's position
    //fly to digital screen
    FlyToTargetPoint(drone,n,C[0],C[1],C[2],0);
  }
  else
  {
    cout<<"time is :"<< end <<endl;
    if(detectednumbers == 10)
    {
      int C[3] = {4,12,2};//the third fixed point C also the digital screen's position
      //fly to digital screen
      FlyToTargetPoint(drone,n,C[0],C[1],C[2],0);
      
    }
    else 
    {
      FlyToTargetPoint(drone,n,A[0],A[1],A[2],0);//judge
    }
    
  }
  */
  
  i = 0;
  //ros::Rate rate(5);
  while(ros::ok() && i < 2)
  {
    ros::spinOnce();
    i = i + 1;
    rate.sleep();
  }

  /*
  i = 0;
  int j;
  // rate ?? smaller than publisher
  while(ros::ok() && i < 15)
  {
    ros::spinOnce();
    j = 0;
    while(ros::ok() && j < 10) // Figure = j
    {
//   X = local.x*cos(yaw0_R) + local.y*sin(yaw0_R);
//   Y = local.x*cos(yaw0_R) - local.y*sin(yaw0_R);
//   north -> body
      if(Number_Position[j][0] > 0.1)
      {
	FigureSum[j][0] = FigureSum[j][0] + Number_Position[j][0] + local.x*cos(yaw0_R) + local.y*sin(yaw0_R);
	FigureSum[j][1] = FigureSum[j][1] + Number_Position[j][1] + local.y*cos(yaw0_R) - local.x*sin(yaw0_R);
	FigureSum[j][2] = FigureSum[j][2] + Number_Position[j][2] + local.z;
// 	Detected = 1;
      }
      if(j == 6)
      {
      cout<<Number_Position[j][0]<<endl;
      cout<<FigureSum[j][0]<<endl;
      cout<<i<<endl<<endl;    
      }
      j = j + 1;
    }
//     if(Detected == 1)
//     {
//       i = i + 1;
//     }
    rate.sleep();
  }
  */
  /*
  j = 0;
  while(ros::ok() && j < 10) // Figure = j
  {
    TargetNumber_Position[j][0] = j;
    TargetNumber_Position[j][1] = FigureSum[j][0] / i;
    TargetNumber_Position[j][2] = FigureSum[j][1] / i;
    TargetNumber_Position[j][3] = FigureSum[j][2] / i;
    if(j == 6){
      cout<<"Number Position (relative to home): "<<endl;
      cout<<TargetNumber_Position[j][1]<<endl;
      cout<<TargetNumber_Position[j][2]<<endl;
      cout<<TargetNumber_Position[j][3]<<endl<<endl;}
    j = j + 1;
  }
  */
  cout<<"Initialization Finished ! "<<endl<<endl;
}
// FigureSum
// 
int TargetFigureRecognization(DJIDrone* drone, ros::NodeHandle n)
{
  FlyToTargetPoint(drone,n,C[0],C[1],C[2],0);
  // fly to target point 
  // target point  = position of big digital screen
  int i = 0;
  //  initial the numbers which will be watched in the future
  while(i < 10)
  {
    bigscreenNumber_Position[i][0] = 0;//1 is detected, 0 is not detected
    bigscreenNumber_Position[i][1] = -1000;//initial the position of numbers
    bigscreenNumber_Position[i][2] = -1000;
    bigscreenNumber_Position[i][3] = -1000;  
    i = i + 1;
  }
  int j = 0;
  while(ros::ok())
  {
    j = 0;
    ros::spinOnce();
    while(ros::ok() && j < 10)
    {
      if(bigscreenNumber_Position[j][1] > 0.1)
      {
	cout<<"ok"<<endl;
	Figure = j;
	return Figure;
      }
      j = j + 1;
    } 
  }
}

void SprayMission(DJIDrone* drone, ros::NodeHandle n, int TargetFigure)
{
  int Continue;
  int i = 1;
  
  double SparyY;
  double SparyZ;
  
// only for YF close aim test: start
/*
  while(i <= 10)
  {
    if( TargetNumber_Position[i][0] == TargetFigure )
    {
      Target_X = TargetNumber_Position[i][1] - Distance2Figure - 0.5;
      Target_Y = TargetNumber_Position[i][2];
      Target_Z = TargetNumber_Position[i][3];
      break;
    }
    i = i + 1;
  }
  
  cout<<"Target Figure = "<<TargetFigure<<endl;
  cout<<"Target X (relative to home) = "<<Target_X<<endl;
  cout<<"Target Y (relative to home) = "<<Target_Y<<endl;
  cout<<"Target Z (relative to home) = "<<Target_Z<<endl;
  
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
*/
// only for YF close aim test: end
  
  
  
//  FlyToTargetPoint(drone,n,Target_X,Target_Y,0.6,0);  
//  sleep(2);
//  cout<<"Water on !"<<endl;
//  gpio_set_value(LEDGPIO, LOW);
//  sleep(2);
  // water off
//  gpio_set_value(LEDGPIO, HIGH);
//  cout<<"Water off !"<<endl;
//  sleep(2);
//  FlyToTargetPoint(drone,n,Target_X,Target_Y,Target_Z,0);
  
//   ros::Rate Aim(20);
//   while(ros::ok())  //mono vision feed back 
//   {
//     ros::spinOnce();
//     drone->velocity_control(0,0,Number_Position[TargetFigure][1]*0.1,Number_Position[TargetFigure][2]*0.1,0);
//     if(Number_Position[TargetFigure][1] < 0.05 && Number_Position[TargetFigure][2] < 0.05)
//     break;
//     Aim.sleep();
//   }
////   cout<<"Move right = "<<Number_Position[TargetFigure][1]<<endl;
////   cout<<"Move up = "<<Number_Position[TargetFigure][2]<<endl<<endl;
////   FlyToTargetPoint(drone,n,Target_X,Target_Y + Number_Position[TargetFigure][1],Target_Z + Number_Position[TargetFigure][2],0);  //float inputx,float inputy,float inputz,float yawT_D 

  
  //1023 YF
  // accurate aiming !!
  cout<<"Accurate aiming started !"<<endl;
  CloseFigureSum[0] = 0; // sum = 0
  CloseFigureSum[1] = 0;
  CloseFigureSum[2] = 0;
  
  i = 0;
  while(i < 2)
  {
    ros::spinOnce();
    i = i + 1;
  }  //read figure image
  
  i = 0;
  while(ros::ok() && i < 5)  // watch the target number in small distance and stay still
  {
    ros::spinOnce();
    d_yaw = (yaw0_R+yawT_R)*R2D - atan2(2.0f*(q.q0*q.q3+q.q1*q.q2), 1-2.0f*(q.q3*q.q3+q.q2*q.q2))*R2D;
    cout<<"OneFigureInSight = "<<OneFigureInSight<<endl;
    if(d_yaw < 3 && OneFigureInSight > 0)
    {
      CloseFigureSum[0] = CloseFigureSum[0] + Number_Position[TargetFigure][1] + local.x*cos(yaw0_R) + local.y*sin(yaw0_R);
      CloseFigureSum[1] = CloseFigureSum[1] + Number_Position[TargetFigure][2] + local.y*cos(yaw0_R) - local.x*sin(yaw0_R);
      CloseFigureSum[2] = CloseFigureSum[2] + Number_Position[TargetFigure][3] + local.z;
      i = i + 1;
    }
  }
  
  AccurateFigurePosi[0] = CloseFigureSum[0] / i - Distance2Figure;
  AccurateFigurePosi[1] = CloseFigureSum[1] / i ;
  AccurateFigurePosi[2] = CloseFigureSum[2] / i + Higher2Figure;
  
  cout<<endl<<AccurateFigurePosi[0]<<endl;
  cout<<AccurateFigurePosi[1]<<endl;
  cout<<AccurateFigurePosi[2]<<endl<<endl;
  sleep(3);
    
  // fly to new watched point
  FlyToTargetPoint(drone,n,AccurateFigurePosi[0],AccurateFigurePosi[1],AccurateFigurePosi[2],0);
  
  i = 0;
  while(i < 2) // watch figure
  {
    ros::spinOnce();
    i = i + 1;
  }  //read figure image
  
  while(ros::ok())  // close enough determination
  {
    
  ros::spinOnce();
  cout<<endl<<"Y = "<<Number_Position[TargetFigure][2]<<endl;
  cout<<"Z = "<<Number_Position[TargetFigure][3]+Higher2Figure<<endl<<endl;
  if( abs(Number_Position[TargetFigure][2]) > FigureCloseEnough  || abs(Number_Position[TargetFigure][3]+Higher2Figure) > FigureCloseEnough) // if not in central area
  {
    // watch again
    cout<<"Not close enough! Watch again! "<<endl;  
    i = 0;

    // Number_Position[TargetFigure][2] + local.y*cos(yaw0_R) - local.x*sin(yaw0_R);
    // Number_Position[TargetFigure][3] + local.z;
    FlyToTargetPoint(drone,n,AccurateFigurePosi[0],Number_Position[TargetFigure][2] + local.y*cos(yaw0_R) - local.x*sin(yaw0_R),Number_Position[TargetFigure][3] + local.z,0);
  }
  else // close enough
  {
    cout<<"Close enough!"<<endl;
    SparyY = local.y*cos(yaw0_R) - local.x*sin(yaw0_R);  //take off coordinatre
    SparyZ = local.z;
    break;
  }
  
  }
  
  //1023 YF
    
  cout<<"Aim Finished !"<<endl;   
  sleep(3);

  ros::Rate close(20);
  while(ros::ok())
  {
    ros::spinOnce();
    //vision accute
    float Y,Z;
    Y = local.y*cos(yaw0_R) - local.x*sin(yaw0_R);
    Z = local.z;
    d_yaw = (yaw0_R+yawT_R)*R2D - atan2(2.0f*(q.q0*q.q3+q.q1*q.q2), 1-2.0f*(q.q3*q.q3+q.q2*q.q2))*R2D;
    
    if(OneFigureInSight == 1)
    {
      drone->velocity_control(0,VtoWall,(SparyY-Y)*KY*0.5,(SparyZ-Z)*KZ*0.5,KA*d_yaw);
    }
    else
    {
      drone->velocity_control(0,VtoWall,0,0,KA*d_yaw);
    }
    
    
    if(DistanceToWall<SprayDistance)
    {
      cout<<"DistanceToWall = "<<DistanceToWall<<endl;
    }
    if(DistanceToWall < SprayDistance  && DistanceToWall > 0.5 )
    {
      drone->velocity_control(1,0,0,0,0);

      // water on
      cout<<"Water on !"<<endl;
      gpio_set_value(LEDGPIO, LOW);
      sleep(2);
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
      //sleep(3);
      drone->velocity_control(1,0,0,0,0);
      //  water on
//       gpio_set_value(LEDGPIO, LOW);
//       cout<<"Water on !"<<endl;
//       sleep(2);
//       // water off
//       gpio_set_value(LEDGPIO, HIGH);
      cout<<"Water off !"<<endl;
      cout<<"Spray Finished ! "<<endl;
      sleep(3);
      break;
    }
    close.sleep();
  }
}


void Landing(ros::NodeHandle n, DJIDrone* drone)
{
  FlyToTargetPoint(drone,n,-1,0,3,0);
   
  sleep(10);
  cout<<"Begin!"<<endl;
  while(1)   
  {
//drone->landing();
ros::spinOnce();
    
      
    if(delt.dx == 160 && delt.dy == 120)
    {
      while(1)
      {
        ros::spinOnce();
	drone->velocity_control(0,0,0,0,0);
	if(delt.dx != 160 || delt.dy != 120)
	  break;
      }      
    }
    else
    {
      while(1)
      {
	ros::spinOnce();
	drone->velocity_control(0,0.2*delt.dy/abs(delt.dy),-0.2*delt.dx/abs(delt.dx),0,0);
	if((abs(delt.dx)*abs(delt.dx)+abs(delt.dy)*abs(delt.dx))< 400 || (delt.dx == 160 && delt.dy == 120))
	{
	  if((abs(delt.dx)*abs(delt.dx)+abs(delt.dy)*abs(delt.dx))< 400)
	    goto here;
	  else
	    break;
	}
      }  
    }
  } 
here:drone->landing();
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
    if( (ros::Time::now().toSec() - StartTime) > 280 )
    {
        return 2; // total time out
    }
    
    return 0;
}


void FlyToTargetPoint(DJIDrone* drone,ros::NodeHandle a,float inputx,float inputy,float inputz,float yawT_D) 
{
  
  int Target_X_north;
  int Target_Y_north;
  
  cout<<"X (forward to home point) is:"<<inputx<<endl;
  cout<<"Y (right to home point) is:"<<inputy<<endl;
  cout<<"Yaw (relative to initial forward) is:"<<yawT_D<<endl<<endl;
  
  yawT_R = yawT_D / 180 * pi; 
  
  int i = 0;
  ros::Rate point(20);
  while(ros::ok() && i < 5)
  {
    ros::spinOnce();
    i = i + 1;
    point.sleep();
  }
  
//   Body_x = local.x*cos(yaw0_R) + local.y*sin(yaw0_R);  //north -> body
//   Body_y = local.y*cos(yaw0_R) - local.x*sin(yaw0_R);

  Target_X_north = inputx*cos(yaw0_R)-inputy*sin(yaw0_R);
  Target_Y_north = inputx*sin(yaw0_R)+inputy*cos(yaw0_R);
  
  d_X = Target_X_north - local.x; // body
  d_Y = Target_Y_north - local.y;
  

  cout<<"X (forward) is:"<<d_X<<endl;
  cout<<"Y (right) is:"<<d_Y<<endl;
  cout<<"Target yaw (relative to body) is: "<<yawT_D<<" deg"<<endl;
  sleep(1);
  cout<<"Move Move Move ! "<<endl;
  
  while(ros::ok())
  {
    ros::spinOnce();
    Target_X_north = inputx*cos(yaw0_R)-inputy*sin(yaw0_R);
    Target_Y_north = inputx*sin(yaw0_R)+inputy*cos(yaw0_R);
  
    d_X = Target_X_north - local.x; // north
    d_Y = Target_Y_north - local.y;
    d_Z = inputz-local.z;
    d_yaw = (yaw0_R+yawT_R)*R2D - atan2(2.0f*(q.q0*q.q3+q.q1*q.q2), 1-2.0f*(q.q3*q.q3+q.q2*q.q2))*R2D;
    
    vx = KX * d_X;
    vy = KY * d_Y;
    vz = KZ * d_Z;
    va = KA * d_yaw;
    
    if(fabs(vx)>V_MAX)
    {
      if(vx>0)
      vx = V_MAX;
      else
      vx = -V_MAX;
    } 
    if(fabs(vy)>V_MAX)
    {
      if(vy>0)
      vy = V_MAX;
      else
      vy = -V_MAX;
    }   
   if(fabs(vz)>VZ_MAX)  //vz 角度
   {
     if(vz>0)
     vz = VZ_MAX;
     else
     vz = -VZ_MAX;
   }     
    if(fabs(va)>VA_MAX)  //va 角度
    {
      if(va>0)
      va = VA_MAX;
      else
      va = -VA_MAX;
    }

    drone->velocity_control(1,vx,vy,vz,va);

    cout<<"d_X is: "<<d_X<<endl;
    cout<<"d_Y is: "<<d_Y<<endl;
    cout<<"d_Z is: "<<d_Z<<endl;
    cout<<"d_yaw is: "<<d_yaw<<endl<<endl;


    if( abs(d_X)<DeltaX && abs(d_Y)<DeltaY && abs(d_Z)<DeltaZ &&  d_yaw<2)  
    {
      i = 0;
      while(ros::ok() && i<10)
      {
	drone->velocity_control(1,0,0,0,0);
	i = i + 1;
      }
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
  for (int i = 0; i < mono_status.size(); i++)  // size biao zhi wei
    {  
        mono_pub::Num num = mono_status.at(i);   
        // ROS_INFO("I heard num: [%d] ", num.id);
	// ROS_INFO("X cordinate of num: [%d] is [%lf]", num.id,-num.position.z);
	// ROS_INFO("Y cordinate of num: [%d] is [%lf]", num.id,num.position.x);
	// ROS_INFO("Z cordinate of num: [%d] is [%lf]", num.id,num.position.y);
	//Number_Position[num.id][0] = 1;
	Number_Position[num.id][1] = -num.position.z;
	Number_Position[num.id][2] = num.position.x;
	Number_Position[num.id][3] = num.position.y;
	bigscreenNumber_Position[num.id][1] = -num.position.z;
	bigscreenNumber_Position[num.id][2] = num.position.x;
	bigscreenNumber_Position[num.id][3] = num.position.y;
	
	OneFigureInSight = mono_status.size();
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

void landingCallback(const guidance::distance::ConstPtr& msg)
{
  delt.dx = msg->dx;
  delt.dy = msg->dy;
  //ROS_INFO("I heard avex:[%f]",msg->avex);
  //ROS_INFO("I heard dx:[%f]",msg->dx);
}


void ultrasonic_callback(sensor_msgs::LaserScan g_ul)  //Guidance SDK
{

  Height = g_ul.ranges[0];
  DistanceToWall = g_ul.ranges[1];
  //cout<<"Height = "<<Height<<endl;
  //cout<<"DistanceToWall = "<<DistanceToWall<<endl;
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

