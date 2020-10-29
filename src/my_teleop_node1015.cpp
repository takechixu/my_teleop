#include "my_teleop/turtlebot3_drive.h"
#include<stdio.h>
#include <termios.h>
#include <unistd.h>

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
  updatecommandVelocity(0.0, 0.0);
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
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

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

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}



int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

bool Turtlebot3Drive::controlLoop(double& xlinear, double& zangular)
{
	if (scan_data_[CENTER] > check_forward_dist_){
		int key = getch();  // 入力キーの値
		switch (key){
			case 'w': 
			    xlinear  +=  0.1;
			    break;
			case 'x':
			    xlinear  += -0.1;
			    break;
			case 'a':
			    zangular +=  0.1;
			    break;
			case 'd':
			    zangular += -0.1;
			    break;
			default:
			    xlinear  = 0.0; 
			    zangular = 0.0; 
			    break;
		}
		updatecommandVelocity(xlinear, zangular);
        }
	
	if(scan_data_[CENTER] < check_forward_dist_){
		if(xlinear > 0.0){
			updatecommandVelocity(0.0, zangular);
		}
		//cout << "auto stop" << endl;
		//prev_tb3_pose_ = tb3_pose_;
		int key = getch();  // 入力キーの値
		switch (key){
			case 'w': 
			    cout << "forward attention !!" << endl;
			    break;
			case 'x':
			    xlinear  += -0.1;
			    break;
			case 'a':
			    zangular +=  0.1;
			    break;
			case 'd':
			    zangular += -0.1;
			    break;
			default:
			    xlinear  = 0.0; 
			    zangular = 0.0; 
			    break;
		}
		updatecommandVelocity(xlinear, zangular);
	}
  return true;
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

    Turtlebot3Drive turtlebot3_drive;
    
    ros::Rate rate(125);
    // ループの頻度を設定するためのオブジェクトを作成。この場合は10Hz、1秒間に10回数、1ループ100ms。

//    geometry_msgs::Twist vel;
    // geometry_msgs::Twist　この型は並進速度と回転速度(vector3:3次元ベクトル) を合わせたもので、速度指令によく使われる

//    pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // マスターにgeometry_msgs::Twist型のデータを送ることを伝える
    // マスターは/cmd_velトピック(1番目の引数）を購読する
    // 全てのノードにトピックができたことを知らせる(advertise)。
    // 2番目の引数はデータのバッファサイズ

    cout << "w: forward, x: backward, d: right, a:left, s:stop" << endl;

    double xlinear=0;
    double zangular=0;
    
    while (ros::ok()) { // このノードが使える間は無限ループする

	turtlebot3_drive.controlLoop(xlinear, zangular);
        ros::spinOnce();     // １回だけコールバック関数を呼び出す
        rate.sleep();        // 指定した周期でループするよう寝て待つ

    }

    return 0;
}

