#include <mc_rtc_ros_control/ROSControlInterface.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/io_utils.h>

namespace mc_rtc_ros_control
{

struct ROSControlInterfaceImpl
{
  ROSControlInterfaceImpl(const std::string & subscribe_to,
                          const std::string & publish_to,
                          const std::vector<std::string> & rjo,
                          double timeStep = 5)
  : rjo_(rjo), timeStep_(timeStep)
  {
    msg_.goal.trajectory.joint_names = rjo;
    msg_.goal.trajectory.points.resize(2);
    msg_.goal.trajectory.points[0].positions.resize(rjo.size(), 0);
    msg_.goal.trajectory.points[1].positions.resize(rjo.size(), 0);
    msg_.goal.trajectory.points[0].velocities.resize(rjo.size(), 0);
    msg_.goal.trajectory.points[1].velocities.resize(rjo.size(), 0);
    pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>(publish_to, 1);
    sub_ = nh_.subscribe(subscribe_to, 1, &ROSControlInterfaceImpl::joint_callback, this);
  }

  void joint_callback(const sensor_msgs::JointState & msg)
  {
    if(!init_done_)
    {
      init(msg);
    }
    else
    {
      run(msg);
    }
  }

  void init(const sensor_msgs::JointState & msg)
  {
    const auto & rjo = rjo_;
    if(msg.name.size() > rjo.size())
    {
      ROS_FATAL("Look like mc_rtc_ros_control is subscribed to a different "
                "robot than what it expects (got: %lu, expected: %lu)",
                msg.name.size(), rjo.size());
    }
    ros_to_rjo_.resize(msg.name.size());
    rjo_to_cmd_.resize(rjo.size(), std::numeric_limits<size_t>::max());
    encoders_.resize(rjo.size(), 0.0);
    velocity_.resize(rjo.size(), 0.0);
    efforts_.resize(rjo.size(), 0.0);
    for(size_t i = 0; i < msg.name.size(); ++i)
    {
      const auto & n = msg.name[i];
      auto it = std::find(rjo.begin(), rjo.end(), n);
      if(it == rjo.end())
      {
        ROS_FATAL_STREAM("Joint state passed in to mc_rtc_ros_control for "
                         << n
                         << " which does not exist in the robot reference joint "
                            "order, something is wrong");
      }
      ros_to_rjo_[i] = std::distance(rjo.begin(), it);
    }
    size_t cmd_index = 0;
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      auto it = std::find(msg.name.begin(), msg.name.end(), rjo[i]);
      if(it != msg.name.end())
      {
        rjo_to_cmd_[i] = cmd_index;
        cmd_index++;
      }
    }
    mc_rtc::log::info("Init done");
    init_done_ = true;
  }

  void run(const sensor_msgs::JointState & msg)
  {
    for(size_t i = 0; i < ros_to_rjo_.size(); ++i)
    {
      auto rjoIdx = ros_to_rjo_[i];
      encoders_[rjoIdx] = msg.position[i];
      velocity_[rjoIdx] = msg.velocity[i];
      if(msg.effort.size())
      {
        efforts_[rjoIdx] = msg.effort[i];
      }
    }
    if(callback_)
    {
      callback_(encoders_, velocity_, efforts_);
    }
  }

  void sendCommand(const std::vector<double> & command)
  {
    static unsigned iter = 0;
    if(iter == 0 || iter % 50 == 0)
    {
      for(size_t i = 0; i < command.size(); ++i)
      {
        msg_.goal.trajectory.points[0].positions[rjo_to_cmd_[i]] = encoders_[i];
        msg_.goal.trajectory.points[0].time_from_start = ros::Duration(0);
        msg_.goal.trajectory.points[1].positions[rjo_to_cmd_[i]] = command[i];
        msg_.goal.trajectory.points[1].time_from_start = ros::Duration(timeStep_);
      }
      mc_rtc::log::info("Current position: {}", mc_rtc::io::to_string(msg_.goal.trajectory.points[0].positions));
      mc_rtc::log::info("Sending command, goal position   : {}", mc_rtc::io::to_string(msg_.goal.trajectory.points[1].positions));
      msg_.header.stamp = ros::Time::now();
      msg_.header.seq++;
      pub_.publish(msg_);
    }
    iter++;
  }

  std::vector<std::string> rjo_;
  double timeStep_ = 0.005;
  bool init_done_ = false;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  control_msgs::FollowJointTrajectoryActionGoal msg_;
  std::vector<size_t> ros_to_rjo_;
  std::vector<size_t> rjo_to_cmd_;
  std::vector<double> encoders_;
  std::vector<double> velocity_;
  std::vector<double> efforts_;

  ROSControlInterface::state_callback_t callback_;
};

ROSControlInterface::ROSControlInterface(const std::string & subscribe_to,
                                         const std::string & publish_to,
                                         const std::vector<std::string> & ref_joint_order)
: impl_(new ROSControlInterfaceImpl(subscribe_to, publish_to, ref_joint_order))
{
}

ROSControlInterface::~ROSControlInterface() = default;

void ROSControlInterface::onStateCallback(state_callback_t callback)
{
  impl_->callback_ = callback;
}

void ROSControlInterface::sendCommand(const std::vector<double> & command)
{
  if(!impl_->init_done_)
  {
    ROS_FATAL("ROSControlInterface::sendCommand called before the receiver was initialized");
  }
  impl_->sendCommand(command);
}

} // namespace mc_rtc_ros_control
