#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

ros::Publisher pos_cmd_pub;
ros::Publisher mav_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
mavros_msgs::PositionTarget mav_cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

bool use_planYaw = true;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
//获取这一组b样条的节点，按照均匀分布的节点，节点数量=阶数+控制点数。  阶数=次数+1
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }
//获取这一组消息中的b样条位置控制点并赋值给pos_pts，3行n列
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }
//创建一个b样条对象pos_traj,构造函数参数为控制点矩阵，b样条阶数，时间步长0.1，即对生成的多项式按t=0.1取值获得对应的值
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj


  Eigen::MatrixXd yaw_pts(1, msg->yaw_pts.size());
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(0, i) = msg->yaw_pts[i];
  }


  UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;


  traj_.clear();
  traj_.push_back(pos_traj);                    //位置
  traj_.push_back(traj_[0].getDerivative());    //速度
  traj_.push_back(traj_[1].getDerivative());    //加速度

  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());


  traj_duration_ = traj_[0].getTimeSum();       //这段轨迹的总时间

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw, yawdot;

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    //当前时间下的位置，速度和加速度
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);


    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ 最后一段轨迹*/
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;


    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];


    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  if(use_planYaw)
  {
    cmd.yaw = yaw;
    cmd.yaw_dot = yawdot;
  }else{
    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;
  }

  pos_cmd_pub.publish(cmd);



  mav_cmd.header.stamp = time_now;
  mav_cmd.header.frame_id = "world"; //表示全局参考坐标系，常见的有odom表示机器人从初始位置开始的相对位移，map用于存储和表示地图数据
  mav_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 北东地，但mavors会自动转为东北天

  mav_cmd.type_mask =  0b0001100000000000;
  mav_cmd.position.x = pos(0);
  mav_cmd.position.y = pos(1);
  mav_cmd.position.z = pos(2);

  mav_cmd.velocity.x = vel(0);
  mav_cmd.velocity.y = vel(1);
  mav_cmd.velocity.z = vel(2);

  mav_cmd.acceleration_or_force.x = acc(0);
  mav_cmd.acceleration_or_force.y = acc(1);
  mav_cmd.acceleration_or_force.z = acc(2);

  mav_cmd.yaw = yaw_yawdot.first;
  mav_cmd.yaw_rate = yaw_yawdot.second;

  last_yaw_ = mav_cmd.yaw;

  mav_cmd_pub.publish(mav_cmd);


}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);   //你的 ROS 节点订阅一个话题时，如果消息发送的速度快于你的节点处理消息的速度，那么未处理的消息将会被存储在一个队列中，直到它们被处理。
                                                                                           //这里的 10 表示最多可以有 10 条未处理的消息被存储在队列中。如果更多的消息在队列满之前到达，那么最旧的消息将被丢弃以腾出空间给新的消息。

  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);           //50 代表了发布者（publisher）的缓存队列大小。当你创建一个发布者时，指定的数字（这里是 50）表示发布者能够缓存未被订阅者接收的消息数量。


  mav_cmd_pub = node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback); // 每0.01s执行一次，这个独立于ros::spin()函数，可以独立于其他线程运行。

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();  //暂停1s

  ROS_WARN("[Traj server]: ready.");

  ros::spin();//ros::spin()调用后不会再返回，也就是主程序到这儿一直循环执行消息回调而不会往下执行后面的代码，而ros::spinOnce()只会调用一次消息回调然后继续执行之后的程序。用ros::spin()时，最好把发布放到回调函数中，这样可以做到每订阅到一条消息就发布一次。



  return 0;
}