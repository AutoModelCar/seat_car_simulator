#ifndef SEAT_CAR_CONTROLLER_H
#define SEAT_CAR_CONTROLLER_H

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

// URDF
#include <urdf/model.h>

// ros_controls
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

namespace seat_car_controller
{
  /**
   * \brief 
   *
   */
  template <class HardwareInterface>
  class SeatCarController : public controller_interface::Controller<HardwareInterface>
  {
    public:
      SeatCarController();
      // public interface inherited from Controller
      bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
      // public interface inherited from ControllerBase 
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);
      void update(const ros::Time& time, const ros::Duration& period);

    private:
      // action subscriber
      ros::Subscriber steering_sub;
      void steering_callback(const std_msgs::Int16::ConstPtr& msg);
      ros::Subscriber speed_sub;
      void speed_callback(const std_msgs::Int16::ConstPtr& msg);
      ros::Subscriber imu_sub;
      void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
      double heading;
      ros::Publisher yaw_pub;
      ros::Publisher twist_pub;
      ros::Publisher steer_angle_pub;

      typedef typename HardwareInterface::ResourceHandleType JointHandle;

      std::string name_;///< Controller name.
      std::vector<JointHandle> joints_;///< Handles to controlled joints.
      std::vector<std::string> joint_names_;///< Controlled joint names.

      typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
      std::vector<PidPtr> pids_;

      double axel_distance;
      double wheel_distance;
      double wheel_diameter;
      double zero_steer_angle;
      std::string steering_topic;
      std::string steering_fb_topic;
      std::string speed_topic;
      std::string yaw_topic;
      std::string twist_topic;

      // joint commands
      double front_left_cmd;
      double front_right_cmd;
      double rear_left_cmd;
      double rear_right_cmd;
      double last_cmd_drive;
      double last_cmd_steer;

      // steering model
      double steer_coeff_a;
      double steer_coeff_b;
      double steer_coeff_c;

      // ROS API
      ros::NodeHandle controller_nh_;
  };

} // namespace

#include <seat_car_controller_impl.h>

#endif // header guard
