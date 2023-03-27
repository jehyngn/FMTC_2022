#include <vector>
#include <cmath>
#include <pure_pursuit_core.h>
#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <algorithm>
#include <tf/transform_broadcaster.h>

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode()
  : private_nh_("~")
  , pp_()
  , LOOP_RATE_(30)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , const_lookahead_distance_(4.0)
  , const_velocity_(3.0)
  , final_constant(1.0)
  , parking_num(1)
{
  initForROS();
}

// Destructor
PurePursuitNode::~PurePursuitNode() {}

// obstacle global variable
float tmp_yaw_rate = 0.0;

bool left_detected = false;
bool left_avoid = false;
bool right_detected = false;
bool right_avoid = false;

bool start_of_mode7_flag = true;

bool construction_flag = false;

bool first_cw = false;
bool second_cw = false;

bool get_x_distance = false;
double delivery_x_distance = 0.0;
bool isPresentYaw = false;
double present_yaw = 0.0;

float target_dist = 3.0;
float min_dist = 1.0;

int obs_cnt = 0;
std::chrono::system_clock::time_point obs_start;

float steering_memory = 0;

bool index_flag = false;

double secDelta = 0.0;
double nsecDelta = 0.0;
double sec = 0.0;
double nsec = 0.0;
double timeDelta = 0.0;
double velocity = 0.0;
double move_distance = 0.0;

bool static_turning = false;

/* traffic Index manager */
int tf_idx_1 = 1000;

const float tf_coord1[2] = {931282.167383, 1929810.61735}; //SNU

// const float tf_coord1[2] = {935572.7866943456, 1915921.7229049096}; //KCITY

int slow_down_tf_idx_1 = 1000;

// Positions where car should slow down before traffic lights
const float slow_down_tf_coord1[2] = {931296.989957, 1929829.77868};

// U-turn index
int construction_idx = 1000;

// K-city 
// const float construction_coord[2] = {935609.1519933953, 1916238.3830293042};

// SNU
const float construction_coord[2] = {931257.8499826894, 1929831.1322850639};

// cross walk index
int cw_idx_1 = 1000;
int cw_idx_2 = 1000;

// SNU
const float cw_coord_1[2] = {931171.100182, 1929664.51083};
const float cw_coord_2[2] = {931157.276563, 1929655.95944};

/* Parking index */
int pk_idx[7] = {0, 0, 0, 0, 0, 0, 0};

// K-City
const float pk_coord[4][2] = { {935539.0990105674, 1915871.7621510457},
                               {935536.7996612387, 1915867.4861468808},
                               {935534.4555976985, 1915863.1177870466},
                               {935532.3626342632, 1915859.1461546447} };



// const float pk_coord[4][2] = { {935539.5063647057, 1915872.2606872066},
//                                {935537.0244854591, 1915867.875155339},
//                                {935534.7374843466, 1915863.454844135},
//                                {935532.3626342632, 1915859.1461546447} };

// SNU
// const float pk_coord[4][2] = { {931347.674500, 1929883.017834},
//                                {931350.691454, 1929886.987260},
//                                {931353.761445, 1929890.945170},
//                                {931356.679894, 1929894.782270} };
// // SNU-2
// const float pk_coord[4][2] = { {931283.899972, 1929822.08646},
//                                {931284.911271, 1929823.376270},
//                                {931284.911271, 1929823.376270},
//                                {931284.911271, 1929823.376270} };


// const float pk_coord1[2] = {935537.101828, 1915867.54744};
// const float pk_coord2[2] = {935534.641037, 1915863.05751};
// const float pk_coord3[2] = {935525.445681, 1915845.94072};
// const float pk_coord4[2] = {935520.003677, 1915835.89874};
// const float pk_coord5[2] = {935517.529914, 1915831.50221};

bool is_parked = false;

int slow_down_before_delivery_idx = 1000;

const float slow_down_coord[2] = {935650.983648, 1916121.33595};

// Delivery Distance
double min_a_dist = 9999999;
double min_b_dist = 9999999;

// Delivery var
double delivery_x_dist;

// max index of pp_.a_cnt array
int a_max_index = -1;
int b_max_index = -1;

// calc max index flag
bool a_cnt_flag = false;
bool b_cnt_flag = false;

// calc show A flag
bool a_show_flag = false;

bool delivery_A_brake_flag = false;
bool delivery_B_brake_flag = false;
bool parking_brake_flag = false;

bool delivery_distance_init_flag = true;

// Parallel Parking flag
bool parking_available_flag = false;
bool once_flag = false;
int check_point_index = 0;
double moved_distance = 0.0;
bool final_parking_flag = false;
int step = 1;

std::vector<int> passed_index;

void PurePursuitNode::initForROS() {
  // ros parameter settings
  private_nh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
  private_nh_.param("const_velocity", const_velocity_, 3.0);
  private_nh_.param("final_constant", final_constant, 1.0);

  nh_.param("vehicle_info/wheel_base", wheel_base_, 1.04);

  ROS_HOME = ros::package::getPath("pure_pursuit");

  // setup subscriber
  pose_sub = nh_.subscribe("current_pose", 1, &PurePursuitNode::callbackFromCurrentPose, this);

  // for main control
  gps_velocity_sub = nh_.subscribe("/gps_velocity", 1, &PurePursuitNode::callbackFromGpsVelocity, this);
  gps_yaw_sub = nh_.subscribe("/gps_yaw", 1, &PurePursuitNode::callbackFromYaw, this);
  static_obstacle_short_sub = nh_.subscribe("/static_obs_flag_short", 1, &PurePursuitNode::callbackFromStaticObstacleShort, this);
  static_obstacle_long_sub = nh_.subscribe("/static_obs_flag_long", 1, &PurePursuitNode::callbackFromStaticObstacleLong, this);
  dynamic_obstacle_short_sub = nh_.subscribe("/dynamic_obs_flag_short", 1, &PurePursuitNode::callbackFromDynamicObstacleShort, this);
  dynamic_obstacle_long_sub = nh_.subscribe("/dynamic_obs_flag_long", 1, &PurePursuitNode::callbackFromDynamicObstacleLong, this);
  parking_rubbercone_sub = nh_.subscribe("/is_parking_rubbercone", 1, &PurePursuitNode::callbackFromParkingRubberCone, this);

  gps_velocity_raw_sub = nh_.subscribe("/gps_front/fix_velocity", 1, &PurePursuitNode::callbackFromGpsVelocityRawdata, this);

  //FMTC
  emergencylight_sub = nh_.subscribe("/yolov7/emergency_light2", 1, &PurePursuitNode::callbackFromEmergencyLight, this);
  traffic_light_sub2 = nh_.subscribe("/yolov7/traffic_light2",1, &PurePursuitNode::callbackFromTrafficLight2, this);

  // setup publisher
  drive_msg_pub = nh_.advertise<race::drive_values>("control_value", 1);
  steering_vis_pub = nh_.advertise<geometry_msgs::PoseStamped>("steering_vis", 1);

  // for visualization
  target_point_pub = nh_.advertise<geometry_msgs::PointStamped>("target_point", 1);
  current_point_pub = nh_.advertise<geometry_msgs::PointStamped>("current_point", 1);
}

void PurePursuitNode::run(char** argv) {
  parking_num = atoi(argv[2]);

  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok()) {
    ros::spinOnce();

    if (!is_waypoint_set_) {
      setPath(argv);
      pp_.setWaypoints(global_path);
    }

    if (!is_pose_set_) {
      loop_rate.sleep();
      continue;
    }

    pp_.setLookaheadDistance(computeLookaheadDistance());

    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa);

    // target point visualization
    publishTargetPointVisualizationMsg();
    publishCurrentPointVisualizationMsg();

    // Traffic Light Index 한번만 초기화 , 신호등 각각 좌표
    if (!index_flag) {
      index_flag = true;
      tf_idx_1 = pp_.getPosIndex(tf_coord1[0], tf_coord1[1]);
      
      slow_down_tf_idx_1 = pp_.getPosIndex(slow_down_tf_coord1[0] , slow_down_tf_coord1[1]);
    
      construction_idx = pp_.getPosIndex(construction_coord[0], construction_coord[1]);

      cw_idx_1 = pp_.getPosIndex(cw_coord_1[0], cw_coord_1[1]);
      cw_idx_2 = pp_.getPosIndex(cw_coord_2[0], cw_coord_2[1]);

      pk_idx[0] = pp_.getPosIndex(pk_coord[0][0], pk_coord[0][1]);
      pk_idx[1] = pp_.getPosIndex(pk_coord[1][0], pk_coord[1][1]);
      pk_idx[2] = pp_.getPosIndex(pk_coord[2][0], pk_coord[2][1]);
      pk_idx[3] = pp_.getPosIndex(pk_coord[3][0], pk_coord[3][1]);

      slow_down_before_delivery_idx = pp_.getPosIndex(slow_down_coord[0], slow_down_coord[1]);
    }

    ROS_INFO("MODE=%d, MISSION_FLAG=%d", pp_.mode, pp_.mission_flag);

    // MODE 0 - 직진
    // MODE 1 - 비상등 우회
    // MODE 2 - 신호등
    // MODE 3 - 동적 장애물
    // MODE 4 - 우회전 
    // MODE 5 - 어린이 보호
    // MODE 6 - 공사중 우회

    // MODE  0 : 직진
    if (pp_.mode == 0) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 9;
      final_constant = 1.0;
    }

    //  MODE 1 : 비상등 우회
    if (pp_.mode == 1) {

      if (pp_.mission_flag == 0 || pp_.mission_flag == 1 || pp_.mission_flag == 2) {
        const_lookahead_distance_ = 6;
        const_velocity_ = 6;
      }

      if (pp_.mission_flag == 0 && pp_.emergencylight_flag) {
        publishPurePursuitDriveMsg(can_get_curvature, kappa, 0.9);

        if (!isPresentYaw) {
          present_yaw = pp_.gps_yaw;
          isPresentYaw = true;
        }

        pp_.mission_flag = 111;
        // for (int i = 0; i < 130/(velocity*3.6); i++) {
        //   pp_.mission_flag = 11;  
        //   pulishControlMsg(6, -22);
        //   usleep(100000);
        // }
        // continue;
      }

      else if (pp_.mission_flag == 111) {
        if (!static_turning) {
          moved_distance = 0.0;
          static_turning = true;
        }
        if (static_turning){
          if (moved_distance < 3.5) {
            pulishControlMsg(7, -22);
            continue;
          }
          else {
            pp_.mission_flag = 11;
            static_turning = false;
          }
        }
      }

      // else if (pp_.mission_flag == 11) {
      //   for (int i = 0; i < 65/(velocity*3.6); i++) {
      //     std::cout << "Turning " << i << "\n";  
      //     pulishControlMsg(6, 22);
      //     usleep(100000);
      //     }
      //     pp_.mission_flag = 1234;
          
      // }
      else if (pp_.mission_flag == 11 && pp_.gps_yaw >= present_yaw + 10) {
        pulishControlMsg(4, 20);
        continue;
      }

      // else if (pp_.mission_flag == 1234) { //&& pp_.gps_yaw < present_yaw - 10) {
      //   const_lookahead_distance_ = 3;
      //   const_velocity_ = 7;
      //   pp_.setWaypoints(avoidance_path);
      //   pp_.mission_flag = 1;
      //   std::cout << "________________yaw2________________\n";
      //   isPresentYaw = false;
      // }
      else if (pp_.mission_flag == 11 && pp_.gps_yaw < present_yaw - 10) {
        const_lookahead_distance_ = 5;
        const_velocity_ = 7;
        pp_.setWaypoints(avoidance_path);
        pp_.mission_flag = 1;
        isPresentYaw = false;
      }

      else if (pp_.mission_flag == 1 && pp_.is_finish) {
        const_lookahead_distance_ = 6;
        const_velocity_ = 7;

        pp_.setWaypoints(global_path);
        pp_.is_finish = false;

        pp_.mission_flag = 2;
      }
    }

    // MODE 2 : 신호등(직진)
    if (pp_.mode == 2) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 10;
      final_constant = 1.0;

      // When traffic lights are RED at slow_down_point -> SLOWNIG DOWN
      if(pp_.reachMissionIdx(slow_down_tf_idx_1) && !pp_.straight_go_flag){
        for (int i = 0; i < 3; i++) {
          publishPurePursuitDriveMsg(can_get_curvature, kappa, 0.05);
          usleep(100000);
        }
      } 
      // When traffic lights are GREEN at slow_down_point -> SPEEDING UP
      else if(pp_.reachMissionIdx(slow_down_tf_idx_1) && pp_.straight_go_flag){
        while(const_velocity_ < 10){
            const_velocity_ += 0.1;
            // pulishControlMsg(const_velocity_ , 0);
            publishPurePursuitDriveMsg(can_get_curvature, kappa);
        }
      }
      // 직진신호등 멈춤
      if (pp_.reachMissionIdx(tf_idx_1) && !pp_.straight_go_flag) { 
        while(!pp_.straight_go_flag)
        {
          publishPurePursuitDriveMsg(can_get_curvature, kappa, 1.0);
          ros::spinOnce();
        }
        continue;
      }
    }

    // MODE 3 : 동적장애물 
    if (pp_.mode == 3) {
      const_velocity_ = 8;
      const_lookahead_distance_ = 6;
      final_constant = 1.0;
      
      if (pp_.mission_flag == 0) {  
        for (int i = 0; i < 3; i++) {
          publishPurePursuitDriveMsg(can_get_curvature, kappa, 0.03);
          usleep(100000);
        }
        pp_.mission_flag = 1;
      }

      else if (pp_.mission_flag == 1) {
        //동적장애물 멀리서 장애물 감지 -> 감속
        while(pp_.is_dynamic_obstacle_detected_long) {
          if (const_velocity_ > 5) {
            const_velocity_ -= 0.1;
            publishPurePursuitDriveMsg(can_get_curvature, kappa);
            ros::spinOnce();
            loop_rate.sleep();
          }
        }
 
        // 동적장애물 멈춰야하는 거리
        while(pp_.is_dynamic_obstacle_detected_short) {
          publishPurePursuitDriveMsg(can_get_curvature, kappa, 1.0);
          ROS_INFO_STREAM("OBSTACLE DETECT");
          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }

    // MODE 4 : 비보호 우회전 및 횡단보도 정지
    if (pp_.mode == 4) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 4;
      const_velocity_ = 7;
      final_constant = 1.3;

      if(pp_.reachMissionIdx(cw_idx_1) && !first_cw){
        for(int i = 0; i < 35; i++){
          publishPurePursuitDriveMsg(can_get_curvature, kappa, 1);
          usleep(50000);
        }
        first_cw = true;
      }

      if(pp_.reachMissionIdx(cw_idx_2) && !second_cw){
        for(int i = 0; i < 35; i++){
          publishPurePursuitDriveMsg(can_get_curvature, kappa, 1);
          usleep(50000);
        }
        second_cw = true;
      }
    }

    // MODE 5 : 어린이 보호 구역 서행
    if (pp_.mode == 5) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 5;
      final_constant = 1.0;
    }

    // MODE 6 : 공사중 우회
    if (pp_.mode == 6) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 6;
      final_constant = 1.2;

      if (pp_.reachMissionIdx(construction_idx) && !construction_flag) {
          for(int i = 0; i < 35; i++){
          publishPurePursuitDriveMsg(can_get_curvature, kappa, 1);
          usleep(50000);
        }
        construction_flag = true;
      }
    }

    // 마지막 waypoint 에 다다랐으면 점차 속도를 줄이기
    if (pp_.is_finish && pp_.mode == 0) {
      while(const_velocity_ > 0) {
        const_velocity_ -= 1;
        pulishControlMsg(const_velocity_,0);
      }
      //마지막 waypoint라면 코드를 종료하기 위함 안될시 삭제요망
      ros::shutdown(); 
      continue;
    }

    publishPurePursuitDriveMsg(can_get_curvature, kappa);

    is_pose_set_ = false;
    loop_rate.sleep();
  }
}


void PurePursuitNode::publishPurePursuitDriveMsg(const bool& can_get_curvature, const double& kappa, const double& brake) {
  double throttle_ = can_get_curvature ? const_velocity_ : 0;
  double steering_radian = convertCurvatureToSteeringAngle(wheel_base_, kappa);
  double steering_ = can_get_curvature ? (steering_radian * 180.0 / M_PI) * -1 * final_constant -1 : 0;
  double brake_ = brake;
  pulishControlMsg(throttle_, steering_, brake_);

  // for steering visualization
  publishSteeringVisualizationMsg(steering_radian);
}

double PurePursuitNode::computeLookaheadDistance() const {
  if (true) {
    return const_lookahead_distance_;
  }
}

void PurePursuitNode::pulishControlMsg(double throttle, double steering, double brake) const {
  race::drive_values drive_msg;
  drive_msg.throttle = throttle;
  drive_msg.steering = steering;
  drive_msg.brake = brake;
  drive_msg_pub.publish(drive_msg);
  steering_memory = drive_msg.steering;
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg) {
  pp_.setCurrentPose(msg);
  is_pose_set_ = true;
}


void PurePursuitNode::setPath(char** argv) {
  std::vector<std::string> paths;
  path_split(argv[1], paths, ",");
  std::ifstream global_path_file(ROS_HOME + "/paths/" + paths[0] + ".txt");

  // path.txt
  // <x, y, mode>
  geometry_msgs::Point p;
  double x, y;
  int mode;

  while(global_path_file >> x >> y >> mode) {
    p.x = x;
    p.y = y;

    global_path.push_back(std::make_pair(p, mode));
  }

  if (paths.size() == 2) {
    std::ifstream avoidance_path_file(ROS_HOME + "/paths/" + paths[1] + ".txt");
    while(avoidance_path_file >> x >> y >> mode) {
      p.x = x;
      p.y = y;
      avoidance_path.push_back(std::make_pair(p, mode));
    }

    // std::ifstream delivery_path_file(ROS_HOME + "/paths/" + paths[2] + ".txt");
    // while(delivery_path_file >> x >> y >> mode) {
    //   p.x = x;
    //   p.y = y;
    //   delivery_path.push_back(std::make_pair(p, mode));
    // }
  }

  // if (paths.size() == 4) {
  //   std::ifstream avoidance_path_file(ROS_HOME + "/paths/" + paths[1] + ".txt");
  //   while(avoidance_path_file >> x >> y >> mode) {
  //     p.x = x;
  //     p.y = y;
  //     avoidance_path.push_back(std::make_pair(p, mode));
  //   }

  //   std::ifstream delivery_path_file(ROS_HOME + "/paths/" + paths[2] + ".txt");
  //   while(delivery_path_file >> x >> y >> mode) {
  //     p.x = x;
  //     p.y = y;
  //     delivery_path.push_back(std::make_pair(p, mode));
  //   }

  //   std::ifstream parallel_parking_path_file(ROS_HOME + "/paths/" + paths[3] + ".txt");
  //   while(parallel_parking_path_file >> x >> y >> mode) {
  //     p.x = x;
  //     p.y = y;
  //     parallel_parking_path.push_back(std::make_pair(p, mode));
  //   }
  // }
  is_waypoint_set_ = true;
}

void PurePursuitNode::publishTargetPointVisualizationMsg() {
  geometry_msgs::PointStamped target_point_msg;
  target_point_msg.header.frame_id = "/base_link";
  target_point_msg.header.stamp = ros::Time::now();
  target_point_msg.point = pp_.getPoseOfNextTarget();
  target_point_pub.publish(target_point_msg);
}

void PurePursuitNode::publishCurrentPointVisualizationMsg() {
  geometry_msgs::PointStamped current_point_msg;
  current_point_msg.header.frame_id = "/base_link";
  current_point_msg.header.stamp = ros::Time::now();
  current_point_msg.point = pp_.getCurrentPose();
  current_point_pub.publish(current_point_msg);
}

void PurePursuitNode::publishSteeringVisualizationMsg (const double& steering_radian) const {
  double yaw = atan2(2.0 * (pp_.current_pose_.orientation.w * pp_.current_pose_.orientation.z + pp_.current_pose_.orientation.x * pp_.current_pose_.orientation.y), 1.0 - 2.0 * (pp_.current_pose_.orientation.y * pp_.current_pose_.orientation.y + pp_.current_pose_.orientation.z * pp_.current_pose_.orientation.z));
  double steering_vis = yaw + steering_radian;

  geometry_msgs::Quaternion _quat = tf::createQuaternionMsgFromYaw(steering_vis);
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/base_link";
  pose.pose.position = pp_.current_pose_.position;
  pose.pose.orientation = _quat;
  steering_vis_pub.publish(pose);
}

void PurePursuitNode::callbackFromGpsVelocity(const std_msgs::Float64& msg) {
  pp_.gps_velocity = msg.data / 3.6;
  moved_distance += pp_.gps_velocity * 0.125;
}

void PurePursuitNode::callbackFromYaw(const std_msgs::Float64& msg) {
  pp_.gps_yaw = msg.data;
}

void PurePursuitNode::callbackFromDynamicObstacleShort(const std_msgs::Bool& msg) {
  pp_.is_dynamic_obstacle_detected_short = msg.data;
}

void PurePursuitNode::callbackFromDynamicObstacleLong(const std_msgs::Bool& msg) {
  pp_.is_dynamic_obstacle_detected_long = msg.data;
}

void PurePursuitNode::callbackFromStaticObstacleShort(const std_msgs::Bool& msg) {
  pp_.is_static_obstacle_detected_short = msg.data;
}

void PurePursuitNode::callbackFromStaticObstacleLong(const std_msgs::Bool& msg) {
  pp_.is_static_obstacle_detected_long = msg.data;
}

void PurePursuitNode::callbackFromParkingRubberCone(const std_msgs::Bool& msg) {
  pp_.is_parking_rubbercone_detected = msg.data;
}

void PurePursuitNode::callbackFromEmergencyLight(const std_msgs::Bool& msg) {
  pp_.emergencylight_flag = msg.data;
}

void PurePursuitNode::callbackFromTrafficLight2(const std_msgs::Bool& msg) {
  pp_.straight_go_flag = msg.data;
}

void PurePursuitNode::callbackFromGpsVelocityRawdata(const geometry_msgs::TwistWithCovarianceStamped& msg) {
  if (sec == 0.0) {
    sec = msg.header.stamp.sec;
    nsec = msg.header.stamp.nsec;
  } 
  else {
    secDelta = msg.header.stamp.sec - sec;
    nsecDelta = (msg.header.stamp.nsec - nsec) / 1000000000.0;
    timeDelta = secDelta + nsecDelta;
    sec = msg.header.stamp.sec;
    nsec = msg.header.stamp.nsec;
  }
  velocity = sqrt(msg.twist.twist.linear.x * msg.twist.twist.linear.x + msg.twist.twist.linear.y * msg.twist.twist.linear.y + msg.twist.twist.linear.z * msg.twist.twist.linear.z);
  move_distance += velocity * timeDelta;

  if (move_distance > 10000)
    move_distance = 0.0;
}

double convertCurvatureToSteeringAngle(const double& wheel_base, const double& kappa) {
  return atan(wheel_base * kappa);
}

void path_split(const std::string& str, std::vector<std::string>& cont, const std::string& delim) {
    size_t prev = 0, pos = 0;
    do {
      pos = str.find(delim, prev);
      if (pos == std::string::npos) pos = str.length();
      std::string token = str.substr(prev, pos-prev);
      if (!token.empty()) cont.push_back(token);
      prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
}

bool compare(darknet_ros_msgs::BoundingBox a, darknet_ros_msgs::BoundingBox b) {
  int a_area = (a.ymax - a.ymin) * (a.xmax - a.xmin);
  int b_area = (b.ymax - b.ymin) * (b.xmax - b.xmin);

  return a_area > b_area ? true : false;
}

bool compare2(vision_distance::Delivery a, vision_distance::Delivery b) {
  return a.dist_y < b.dist_y ? true : false;
}

}  // namespace waypoint_follower