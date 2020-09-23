// HIT Algricutural
// ISAP All Right Reserved 2017.10

#include "StateMachine.h" 

int main ( int argc, char ** argv ) 
{
  
  ros::init(argc,argv,"StateMachineNode");
  
  ros::NodeHandle n;
  DJIDrone* drone = new DJIDrone(n);

  ros::Subscriber local_position = n.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, local_position_callback);
  ros::Subscriber quaternion = n.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 10, AttitudeQuaternion_callback);
  ros::Subscriber flight_status = n.subscribe<std_msgs::UInt8>("dji_sdk/flight_status",10,flight_status_callback);
  ros::Subscriber ultrasonic_height_sub  = n.subscribe("/guidance/ultrasonic", 10, ultrasonic_callback);  //超声高度/距离
  ros::Subscriber sub = n.subscribe<mono_pub::MonoMsg>("monodata", 1000, messageCallback);  // Mono Camera data
  ros::Subscriber delt = n.subscribe<guidance::distance>("/guidance/relative",10,landingCallback);
  MissionStart = 0;
  MissionOver = 0;
  TimeOut = 0;

  mode = 0;
  
  drone->request_sdk_permission_control();
  printf("Ctrl State = %d \n",drone->request_sdk_permission_control());
  
  MissionStart = Takeoff(drone);      
  //keep waiting until taking off and start mission
  cout<<"Mission Start = "<<MissionStart<<endl;  
  
  TimeOut = 1;  //only for testing
  
  while(ros::ok())//MissionStart == 1) //***
  {
    
//     TimeOut = Timer();
//     if(TimeOut == 2)  //total time out
//     {
//       mode = 3;
//     }
//     else if(TimeOut == 1) //single mission time out
//     {
//       mode = 1;  
//       // abort current mission and recognize next figure
//     }
    
    switch(mode)
    {
      
      case 0:
	printf("Mode: Initialization\n");
	// Initialization(drone, n); // only for YF close aim test
	mode = 1;
	break;
	
      case 1:
	printf("Mode: Target Figure Recognization\n");
	TargetFigure = 7; // only for YF close aim test //TargetFigureRecognization(drone, n); // watch big digital screen  
	printf("TargetFigure: %d \n", TargetFigure);
	mode = 2;
	break;
	
      case 2:
	printf("Mode: Spray on Target Figure\n");
	SprayMission(drone, n, TargetFigure);
	printf("Spraying on %d finished !\n", TargetFigure);
	if(TimeOut == 1){
	  mode = 3;}
	else{
	  mode = 1;}
	break;
	
      case 3:
	printf("Mode: Returning to Home Point and Landing\n");
	// Landing(n, drone);
	MissionOver = 1;
	break;
	
    }
    
    if(MissionOver == 1){
      printf("Mission Over !/n");
      break;}
  }
 
  return(0);
}
