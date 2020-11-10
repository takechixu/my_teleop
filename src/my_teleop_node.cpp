#include "my_teleop/turtlebot3_drive.h"

using namespace std;

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  ROS_ASSERT(init());
}

Turtlebot3Drive::~Turtlebot3Drive()
{
//  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.5;
  check_side_dist_    = 0.3;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);
  joy_sub_ = nh_.subscribe("joy", 10, &Turtlebot3Drive::updatecommandVelocity, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}


void Turtlebot3Drive::updatecommandVelocity(const sensor_msgs::Joy& joy_msg)
{
  geometry_msgs::Twist cmd_vel;

  if(scan_data_[CENTER] > check_forward_dist_){
	if (scan_data_[LEFT] < check_side_dist_){
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = -0.5;
		cout << "right" << endl;
	}
	else if (scan_data_[RIGHT] < check_side_dist_){
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0.5;
		cout << "left" << endl;
	}
	else{
		cmd_vel.linear.x = (joy_msg.axes[4]-1)*(-0.25)+(joy_msg.axes[3]-1)*(0.25);
		cmd_vel.angular.z = joy_msg.axes[0]*0.8;
		cout << "free" << endl;
	}
  }

  else if(scan_data_[CENTER] < check_forward_dist_){
	if (scan_data_[LEFT] > check_side_dist_){
		cmd_vel.linear.x = (joy_msg.axes[3]-1)*(0.25);
		cmd_vel.angular.z = 0.5;
		cout << "auto left" << endl;
	}
	else if (scan_data_[RIGHT] > check_side_dist_){
		cmd_vel.linear.x = (joy_msg.axes[3]-1)*(0.25);
		cmd_vel.angular.z = -0.5;
		cout << "auto right" << endl;
	}
	else{
		cmd_vel.linear.x = (joy_msg.axes[3]-1)*(0.25);
		cmd_vel.angular.z = 0;
		cout << "back" << endl;
	}
//	cmd_vel.linear.x = (joy_msg.axes[3]-1)*(0.25);
//	cmd_vel.angular.z = joy_msg.axes[0]*0.8;
  }
  


  cmd_vel_pub_.publish(cmd_vel);
}


int main(int argc, char **argv)
{

//←コードのコメントアウト
    //←説明文


    ros::init(argc, argv, "my_teleop_node");
    // initでROSを初期化し、my_teleop_nodeという名前をノードにつける
    // 同じ名前のノードが複数あるとだめなので、ユニークな名前をつける

//    ros::NodeHandle nh; // ノードハンドラの作成。ハンドラは必要時に起動される。

//    ros::Publisher  pub; // パブリッシャの作成。トピックに対してデータを送信。

//    ros::Subscriber joy_sub = nh.subscribe("joy", 10, Turtlebot3Drive::updatecommandVelocity);
    Turtlebot3Drive turtlebot3_drive;
    
    ros::Rate rate(10);
    // ループの頻度を設定するためのオブジェクトを作成。この場合は10Hz、1秒間に10回数、1ループ100ms。

//    geometry_msgs::Twist vel;

//    pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // マスターにgeometry_msgs::Twist型のデータを送ることを伝える
    // マスターは/cmd_velトピック(1番目の引数）を購読する
    // 全てのノードにトピックができたことを知らせる(advertise)。
    // 2番目の引数はデータのバッファサイズ
    
    while (ros::ok()) { // このノードが使える間は無限ループする

        ros::spinOnce();     // １回だけコールバック関数を呼び出す
        rate.sleep();        // 指定した周期でループするよう寝て待つ

    }

    return 0;
}

