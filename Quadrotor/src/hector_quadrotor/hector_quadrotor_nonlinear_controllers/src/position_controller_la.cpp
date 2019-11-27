//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <controller_interface/controller.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <limits>

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <cmath>
#include <pluginlib/class_list_macros.h>

#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/limiters.h>

#include <std_msgs/Bool.h>
#include "Eigen/Dense"

namespace hector_quadrotor_controllers
{

using namespace hector_quadrotor_interface;
using namespace Eigen;

class PositionController : public controller_interface::Controller<hector_quadrotor_interface::QuadrotorInterface>
{
public:
  PositionController()
    : pose_command_valid_(false)
  {
  }

  virtual ~PositionController()
  {
    pose_subscriber_.shutdown();
  }
  virtual bool init(hector_quadrotor_interface::QuadrotorInterface *interface, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh)
  {
    // get interface handles
    pose_ = interface->getPose();
    twist_ = interface->getTwist();
    accel_ = interface->getAccel();
    motor_status_ = interface->getMotorStatus();

    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    root_nh.param<std::string>("world_frame", world_frame_, "/world");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

    getMassAndInertia(root_nh, mass_, inertia_);

    // resolve frames
    tf_prefix_ = tf::getPrefixParam(root_nh);
    world_frame_ = tf::resolve(tf_prefix_, world_frame_);
    base_link_frame_ = tf::resolve(tf_prefix_, base_link_frame_);
    base_stabilized_frame_ = tf::resolve(tf_prefix_, base_stabilized_frame_);

    // Setup pose visualization marker output
    initMarker(root_nh.getNamespace());
    marker_publisher_ = root_nh.advertise<visualization_msgs::Marker>("command/pose_marker", 1);

    // Initialize inputs/outputs
    pose_input_ = interface->addInput<PoseCommandHandle>("pose");
    wrench_output_ = interface->addOutput<WrenchCommandHandle>("wrench");

    // subscribe to commanded pose and velocity
    pose_subscriber_ = root_nh.subscribe<geometry_msgs::PoseStamped>("command/pose", 1, boost::bind(
        &PositionController::poseCommandCallback, this, _1));

    return true;
  }

  void reset()
  {
    // Set commanded pose to robot's current pose
    updatePoseCommand(pose_->pose());
    wrench_control_ = geometry_msgs::WrenchStamped();
    pose_command_valid_ = false;
  }

  virtual void starting(const ros::Time &time)
  {
    reset();
    wrench_output_->start();
  }

  virtual void stopping(const ros::Time &time)
  {
    wrench_output_->stop();
    pose_command_valid_ = false;
  }

  void poseCommandCallback(const geometry_msgs::PoseStampedConstPtr &command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ros::Time start_time = command->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);

    updatePoseCommand(*command);
  }

  virtual void update(const ros::Time &time, const ros::Duration &period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    // Get pose command command input
    if (pose_input_->connected() && pose_input_->enabled())
    {
      updatePoseCommand(pose_input_->getCommand());
    }

   // Check if motors are running
    if (motor_status_->motorStatus().running == false) {
      if (pose_command_valid_) {
        ROS_INFO_NAMED("position_controller", "Disabled position control while motors are not running.");
      }
      pose_command_valid_ = false;
    }

    // Abort if no pose command is available
    if (!pose_command_valid_) {
      reset();
      wrench_output_->stop();
      return;
    } else {
      wrench_output_->start();
    }

    Pose pose = pose_->pose();

    double yaw_command;
    {
      tf2::Quaternion q;
      double temp;
      tf2::fromMsg(pose_command_.orientation, q);
      tf2::Matrix3x3(q).getRPY(temp, temp, yaw_command);
    }

    double sin_yaw, cos_yaw;
    sincos(yaw_command, &sin_yaw, &cos_yaw);

    Twist twist = twist_->twist(), twist_body;
    twist_body.linear = pose_->toBody(twist.linear);
    twist_body.angular = pose_->toBody(twist.angular);

    const double gravity = 9.8065;
    // period.toSec()
    Vector3d ep, ev, eR, ew, zw, zB,  zB_des, yB_des, xB_des, xC_des, Fdes, u;
    Matrix3d Rdes,  R, Re;
    Affine3d  T;

    DiagonalMatrix<double, 3> Kp(6, 6, 15);
    DiagonalMatrix<double, 3> Kv(3, 3, 7.5);
    DiagonalMatrix<double, 3> KR(1, 1, 0.2);
    DiagonalMatrix<double, 3> Kw(0.05, 0.05, 0.1);

    tf2::fromMsg(pose, T);
    R = T.rotation();

    zB = R.col(2);
    zw << 0, 0, 1;

    ep << pose_command_.position.x - pose.position.x, pose_command_.position.y - pose.position.y, pose_command_.position.z - pose.position.z;
    ev <<  -twist.linear.x, -twist.linear.y, -twist.linear.z;
    Fdes =mass_ * gravity*zw + Kp*ep +  Kv*ev;

    xC_des << cos_yaw, sin_yaw , 0;
    zB_des = Fdes / Fdes.norm();
    yB_des = zB_des.cross(xC_des);
    yB_des = yB_des / yB_des.norm();
    xB_des = yB_des.cross(zB_des);

    Rdes.col(0) = xB_des;
    Rdes.col(1) = yB_des;
    Rdes.col(2) = zB_des;

    Re = 0.5*(Rdes.transpose()* R - R.transpose()*Rdes);

    eR << -Re(2,1), -Re(0,2), -Re(1,0);
    ew << -twist_body.angular.x, -twist_body.angular.y, -twist_body.angular.z;
    u = KR*eR + Kw*ew;

    // set wrench output
    wrench_control_.wrench.torque.x =  u(0) ;
    wrench_control_.wrench.torque.y =  u(1);
    wrench_control_.wrench.torque.z =  u(2);

    wrench_control_.wrench.force.x  = 0.0;
    wrench_control_.wrench.force.y  = 0.0;
    wrench_control_.wrench.force.z =   Fdes.dot(zB);

    if (wrench_control_.wrench.force.z > 30.0) wrench_control_.wrench.force.z = 30.0;
    if (wrench_control_.wrench.force.z <   0.0) wrench_control_.wrench.force.z =   0.0;

    if (wrench_control_.wrench.torque.x > 10.0) wrench_control_.wrench.torque.x = 10.0;
    if (wrench_control_.wrench.torque.x < -10.0) wrench_control_.wrench.torque.x = -10.0;

    if (wrench_control_.wrench.torque.y > 10.0) wrench_control_.wrench.torque.y = 10.0;
    if (wrench_control_.wrench.torque.y < -10.0) wrench_control_.wrench.torque.y = -10.0;

    if (wrench_control_.wrench.torque.z > 1.0) wrench_control_.wrench.torque.z = 1.0;
    if (wrench_control_.wrench.torque.z < -1.0) wrench_control_.wrench.torque.z = -1.0;

    wrench_control_.header.stamp = time;
    wrench_control_.header.frame_id = base_link_frame_;
    wrench_output_->setCommand(wrench_control_.wrench);
  }

private:
  void updatePoseCommand(const geometry_msgs::PoseStamped &new_pose)
  {
    // TODO TF to world frame
    if (new_pose.header.frame_id != world_frame_) {
      ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "position_controller", "Pose commands must be given in the " << world_frame_ << " frame, ignoring command");
    }
    else
    {
      updatePoseCommand(new_pose.pose);
    }
  }

  void updatePoseCommand(const geometry_msgs::Pose &new_pose)
  {
    {
      pose_command_.position = new_pose.position;
      // Strip non-yaw components from orientation
      tf2::Quaternion q;
      double roll, pitch, yaw;
      tf2::fromMsg(new_pose.orientation, q);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      q.setRPY(0, 0, yaw);
      pose_command_.orientation = tf2::toMsg(q);
      pose_command_valid_ = true;
    }
    pose_marker_.pose = pose_command_;
    marker_publisher_.publish(pose_marker_);
  }

  void initMarker(std::string name)
  {
    pose_marker_.header.frame_id = world_frame_;
    pose_marker_.ns = name;
    pose_marker_.id = 0;
    pose_marker_.type = visualization_msgs::Marker::ARROW;
    pose_marker_.scale.x = 0.15;
    pose_marker_.scale.y = pose_marker_.scale.z = 0.03;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.g = 0.5;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.a = 1.0;
  }

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr accel_;
  MotorStatusHandlePtr motor_status_;

  PoseCommandHandlePtr pose_input_;
  WrenchCommandHandlePtr wrench_output_;

  ros::Subscriber pose_subscriber_;
  ros::Publisher marker_publisher_;

  visualization_msgs::Marker pose_marker_;

  geometry_msgs::Pose pose_command_;
  geometry_msgs::WrenchStamped wrench_control_;
  bool pose_command_valid_ ;

  std::string base_link_frame_, base_stabilized_frame_, world_frame_;
  std::string tf_prefix_;

  double mass_;
  double inertia_[3];

  boost::mutex command_mutex_;
};

} // namespace hector_quadrotor_controllers

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::PositionController, controller_interface::ControllerBase)
