#ifndef SEAT_CAR_CONTROLLER_IMP_H
#define SEAT_CAR_CONTROLLER_IMP_H

#include <limits>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <tf/tf.h>

namespace seat_car_controller
{
  namespace internal
  {
    boost::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
    {
      boost::shared_ptr<urdf::Model> urdf(new urdf::Model);

      std::string urdf_str;
      // Check for robot_description in proper namespace
      if (nh.getParam(param_name, urdf_str))
      {
	if (!urdf->initString(urdf_str))
	{
	  ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
	      nh.getNamespace() << ").");
	  return boost::shared_ptr<urdf::Model>();
	}
      }
      // Check for robot_description in root
      else if (!urdf->initParam("robot_description"))
      {
	ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
	return boost::shared_ptr<urdf::Model>();
      }
      return urdf;
    }

    typedef boost::shared_ptr<const urdf::Joint> UrdfJointConstPtr;
    std::vector<UrdfJointConstPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names)
    {
      std::vector<UrdfJointConstPtr> out;
      for (unsigned int i = 0; i < joint_names.size(); ++i)
      {
	UrdfJointConstPtr urdf_joint = urdf.getJoint(joint_names[i]);
	if (urdf_joint)
	{
	  out.push_back(urdf_joint);
	}
	else
	{
	  ROS_ERROR_STREAM("Could not find joint '" << joint_names[i] << "' in URDF model.");
	  return std::vector<UrdfJointConstPtr>();
	}
      }
      return out;
    }

    std::string getNamespace(const ros::NodeHandle& nh)
    {
      const std::string complete_ns = nh.getNamespace();
      std::size_t id   = complete_ns.find_last_of("/");
      return complete_ns.substr(id + 1);
    }

  } // namespace

  template <class HardwareInterface>
    inline void SeatCarController<HardwareInterface>::starting(const ros::Time& time)
    {
      // Reset PIDs, zero effort commands
      for (unsigned int i = 0; i < this->pids_.size(); ++i)
      {
        this->pids_[i]->reset();
        this->joints_[i].setCommand(0.0);
      }
      this->front_left_cmd=0;
      this->front_right_cmd=0;
      this->rear_left_cmd=0;
      this->rear_right_cmd=0;

      ::srand(::time(NULL));
    }

  template <class HardwareInterface>
    inline void SeatCarController<HardwareInterface>::stopping(const ros::Time& time)
    {
    }

  template <class HardwareInterface>
    SeatCarController<HardwareInterface>::SeatCarController()
    {
    }

  template <class HardwareInterface>
    bool SeatCarController<HardwareInterface>::init(HardwareInterface* hw,ros::NodeHandle&   root_nh,ros::NodeHandle&   controller_nh)
    {
      std::string joint_name;

      // Cache controller node handle
      this->controller_nh_ = controller_nh;
      // Controller name
      this->name_ = internal::getNamespace(this->controller_nh_);

      // get the joint names
      this->joint_names_.clear();
      joint_name="";
      controller_nh_.getParam("steer_left_joint", joint_name);
      if(joint_name=="")
      {
        ROS_ERROR_STREAM_NAMED(name_, "steer_left_joint not defined");
        return false;
      }
      else
        this->joint_names_.push_back(joint_name);
      joint_name="";
      controller_nh_.getParam("steer_right_joint", joint_name);
      if(joint_name=="")
      {
        ROS_ERROR_STREAM_NAMED(name_, "steer_right_joint not defined");
        return false;
      }
      else
        this->joint_names_.push_back(joint_name);
      joint_name="";
      controller_nh_.getParam("drive_rear_left_joint", joint_name);
      if(joint_name=="")
      {
        ROS_ERROR_STREAM_NAMED(name_, "drive_rear_left_joint not defined");
        return false;
      }
      else
        this->joint_names_.push_back(joint_name);
      joint_name="";
      controller_nh_.getParam("drive_rear_right_joint", joint_name);
      if(joint_name=="")
      {
        ROS_ERROR_STREAM_NAMED(name_, "drive_right_joint not defined");
        return false;
      }
      else
        this->joint_names_.push_back(joint_name);
      joint_name="";
      controller_nh_.getParam("drive_front_left_joint", joint_name);
      if(joint_name=="")
      {
        ROS_ERROR_STREAM_NAMED(name_, "drive_front_left_joint not defined");
        return false;
      }
      else
        this->joint_names_.push_back(joint_name);
      joint_name="";
      controller_nh_.getParam("drive_front_right_joint", joint_name);
      if(joint_name=="")
      {
        ROS_ERROR_STREAM_NAMED(name_, "drive_front_right_joint not defined");
        return false;
      }
      else
        this->joint_names_.push_back(joint_name);
      const unsigned int n_joints = joint_names_.size();
      // get the joints from the urdf file
      boost::shared_ptr<urdf::Model> urdf = internal::getUrdf(root_nh, "robot_description");
      if (!urdf)
      {
        ROS_ERROR_STREAM_NAMED(name_, "No robot description found");
        return false;
      }
      std::vector<internal::UrdfJointConstPtr> urdf_joints = internal::getUrdfJoints(*urdf, this->joint_names_);
      if (urdf_joints.empty())
      {
        ROS_ERROR_STREAM_NAMED(name_, "Desired joint not found in the robot model");
        return false;
      }
      // Initialize the joints
      this->joints_.resize(n_joints);
      this->pids_.resize(n_joints);
      for (unsigned int i = 0; i < n_joints; ++i)
      {
	// Joint handle
	try {
          this->joints_[i] = hw->getHandle(this->joint_names_[i]);
          
          // Node handle to PID gains
          ros::NodeHandle joint_nh(controller_nh_, std::string("gains/") + this->joints_[i].getName());
          // Init PID gains from ROS parameter server
          this->pids_[i].reset(new control_toolbox::Pid());
          if (!this->pids_[i]->init(joint_nh))
          {
            ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
            return false;
          }
        }catch (...){
	  ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << this->joint_names_[i] << "' in '" <<
	      this->getHardwareInterfaceType() << "'.");
	  return false;
	}
      }
      // get the car parameters
      this->axel_distance=0.26;
      controller_nh_.getParam("axel_distance", this->axel_distance);
      ROS_DEBUG_STREAM_NAMED(name_, "Axel distance set to: " << this->axel_distance);

      this->wheel_distance=0.165;
      controller_nh_.getParam("wheel_distance", this->wheel_distance);
      ROS_DEBUG_STREAM_NAMED(name_, "Wheel distance set to: " << this->wheel_distance);

      this->wheel_diameter=0.063;
      controller_nh_.getParam("wheel_diameter", this->wheel_diameter);
      ROS_DEBUG_STREAM_NAMED(name_, "Wheel diameter set to: " << this->wheel_diameter);

      this->zero_steer_angle=0.0;
      controller_nh_.getParam("zero_steer_angle", this->zero_steer_angle);
      ROS_DEBUG_STREAM_NAMED(name_, "Zero steer angle set to: " << this->zero_steer_angle);

      this->steering_topic="/steering";
      controller_nh_.getParam("steering_topic", this->steering_topic);
      ROS_DEBUG_STREAM_NAMED(name_, "Steering topic set to: " << this->steering_topic);

      this->steering_fb_topic="/steering_angle";
      controller_nh_.getParam("steering_fb_topic", this->steering_fb_topic);
      ROS_DEBUG_STREAM_NAMED(name_, "Steering feedback topic set to: " << this->steering_fb_topic);

      this->speed_topic="/manual_control/speed";
      controller_nh_.getParam("speed_topic", this->speed_topic);
      ROS_DEBUG_STREAM_NAMED(name_, "Speed topic set to: " << this->speed_topic);

      this->yaw_topic="/model_car/yaw";
      controller_nh_.getParam("yaw_topic", this->yaw_topic);
      ROS_DEBUG_STREAM_NAMED(name_, "Yaw topic set to: " << this->yaw_topic);

      this->twist_topic="/model_car/twist";
      controller_nh_.getParam("twist_topic", this->twist_topic);
      ROS_DEBUG_STREAM_NAMED(name_, "Twist topic set to: " << this->twist_topic);

      this->steer_coeff_a=0.0;
      controller_nh_.getParam("steer_coeff_a", this->steer_coeff_a);
      ROS_DEBUG_STREAM_NAMED(name_, "Steering model coefficient A set to: " << this->steer_coeff_a);

      this->steer_coeff_b=0.0;
      controller_nh_.getParam("steer_coeff_b", this->steer_coeff_b);
      ROS_DEBUG_STREAM_NAMED(name_, "Steering model coefficient B set to: " << this->steer_coeff_b);

      this->steer_coeff_c=0.0;
      controller_nh_.getParam("steer_coeff_c", this->steer_coeff_c);
      ROS_DEBUG_STREAM_NAMED(name_, "Steering model coefficient C set to: " << this->steer_coeff_c);

      // ros communications
      this->steering_sub = root_nh.subscribe(this->steering_topic, 1, &SeatCarController<HardwareInterface>::steering_callback,this);
      this->speed_sub = root_nh.subscribe(this->speed_topic, 1, &SeatCarController<HardwareInterface>::speed_callback,this);
      this->imu_sub = root_nh.subscribe("/imu/data", 1, &SeatCarController<HardwareInterface>::imu_callback,this);

      ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
	  "\n- Number of joints: " << joints_.size() <<
	  "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'");

      this->yaw_pub = root_nh.advertise<std_msgs::Float32>(this->yaw_topic,1);
      this->twist_pub = root_nh.advertise<geometry_msgs::Twist>(this->twist_topic,1);
      this->steer_angle_pub = root_nh.advertise<std_msgs::UInt8>(this->steering_fb_topic,1);

      this->last_cmd_drive=0.0;
      this->last_cmd_steer=0.0;
      this->heading=0.0;

      return true;
    }

  template <class HardwareInterface>
    void SeatCarController<HardwareInterface>::update(const ros::Time& time, const ros::Duration& period)
    {
      double error,command,error_v,servo_angle;
      double steer_l_pos,steer_r_pos,drive_r_l_pos,drive_r_r_pos,drive_f_l_pos,drive_f_r_pos;
      double steer_l_vel,steer_r_vel,drive_r_l_vel,drive_r_r_vel,drive_f_l_vel,drive_f_r_vel;
      static double yaw=0.0;
      static double last_steer_angle=0.0;
      geometry_msgs::Twist twist_msg;
      std_msgs::UInt8 steer_angle_msg;

      steer_l_pos=this->joints_[0].getPosition();
      steer_r_pos=this->joints_[1].getPosition();
      drive_r_l_pos=this->joints_[2].getPosition();
      drive_r_r_pos=this->joints_[3].getPosition();
      drive_f_l_pos=this->joints_[4].getPosition();
      drive_f_r_pos=this->joints_[5].getPosition();
      steer_l_vel=this->joints_[0].getVelocity();
      steer_r_vel=this->joints_[1].getVelocity();
      drive_r_l_vel=this->joints_[2].getVelocity();
      drive_r_r_vel=this->joints_[3].getVelocity();
      drive_f_l_vel=this->joints_[4].getVelocity();
      drive_f_r_vel=this->joints_[5].getVelocity();

      // Update PIDs and send command to the motors
      error=this->front_left_cmd-steer_l_pos;
      command=pids_[0]->computeCommand(error,period);
      this->joints_[0].setCommand(command);
      error=this->front_right_cmd-steer_r_pos;
      command=pids_[1]->computeCommand(error,period);
      this->joints_[1].setCommand(command);
      double cotan_steer;
      double radius_1,radius_2,radius;
      double steer_angle;
      double f_speed;
      double yaw_inc;
      if(steer_l_pos*steer_r_pos<0.0)
      {
        cotan_steer=(1/tan(+steer_l_pos)+1/tan(-steer_r_pos))/2;
        steer_angle=atan2(1.0,cotan_steer);
        if(steer_angle>(3.14159/2.0))
          steer_angle-=3.14159;
        if(steer_angle>0.4)
          steer_angle=0.4;
        else if(steer_angle<-0.4)
          steer_angle=-0.4;
      }
      else 
        steer_angle=0.0;
      if(fabs(steer_angle)>0.001)
      {
        radius=fabs(this->axel_distance*cotan_steer);
        f_speed=(-drive_r_l_vel*this->wheel_diameter/2.0+drive_r_r_vel*this->wheel_diameter/2.0)/2.0;
        if(steer_r_pos>steer_l_pos)
          yaw_inc=f_speed*period.toSec()/radius;
        else
          yaw_inc=-f_speed*period.toSec()/radius;
      }
      else
      {
        radius=std::numeric_limits<double>::max();
        steer_angle=0.0;
        f_speed=(-drive_r_l_vel*this->wheel_diameter/2.0+drive_r_r_vel*this->wheel_diameter/2.0)/2.0;
        yaw_inc=0.0;
      }
      yaw-=yaw_inc;
      if(yaw>3.14159)
        yaw-=2*3.14159;
      else if(yaw<-3.14159)
        yaw+=2*3.14159;
      error=this->rear_left_cmd-drive_r_l_vel;
      command=pids_[2]->computeCommand(error,period);
      this->joints_[2].setCommand(command);
      error=this->rear_right_cmd-drive_r_r_vel;
      command=pids_[3]->computeCommand(error,period);
      this->joints_[3].setCommand(command);
      error=this->rear_left_cmd-drive_f_l_vel;
      command=pids_[4]->computeCommand(error,period);
      this->joints_[4].setCommand(command);
      error=this->rear_right_cmd-drive_f_r_vel;
      command=pids_[5]->computeCommand(error,period);
      this->joints_[5].setCommand(command);

      // compute and publish yaw and twist
      twist_msg.linear.x=-((drive_r_l_vel-drive_r_r_vel)*5.5)/2.0;
      twist_msg.linear.y=0.0;
      twist_msg.linear.z=0.0;
      twist_msg.angular.x=0.0;
      twist_msg.angular.y=0.0;
      twist_msg.angular.z=0.0;

      this->twist_pub.publish(twist_msg);

      servo_angle=-4.0*steer_angle+3.14159/2.0;
      steer_angle_msg.data=180+(unsigned short int)((servo_angle*180.0)/3.14159);
      this->steer_angle_pub.publish(steer_angle_msg);
    }

  template <class HardwareInterface>
    void SeatCarController<HardwareInterface>::steering_callback(const std_msgs::Int16::ConstPtr& msg)
    {
      double servo_angle=(msg->data-this->zero_steer_angle)*3.14159/180.0;
      double car_angle;
      double radius;

      car_angle=-((servo_angle-3.14159/2.0)/4.0);
      if(fabs(car_angle)>0.001)
        radius=fabs(this->axel_distance/tan(car_angle));
      else
        radius=std::numeric_limits<double>::max();

      ROS_DEBUG("SeatCarController: SteeringCmd=%d, ServoAngle=%f, CarAngle=%f, Radius=%f", msg->data, servo_angle, car_angle, radius);
      //std::cout << msg->data << "," << servo_angle << "," << car_angle << "," << radius << std::endl;

      if(car_angle>0)
      {
        // right inner
        this->front_right_cmd=-atan2(this->axel_distance,radius-this->wheel_distance/2.0);
        // left outer
        this->front_left_cmd=atan2(this->axel_distance,radius+this->wheel_distance/2.0);
      }
      else
      {
        // right outer
        this->front_right_cmd=atan2(this->axel_distance,radius+this->wheel_distance/2.0);
        // left inner
        this->front_left_cmd=-atan2(this->axel_distance,radius-this->wheel_distance/2.0);
      }

      this->last_cmd_steer=car_angle;
    }

  template <class HardwareInterface>
    void SeatCarController<HardwareInterface>::speed_callback(const std_msgs::Int16::ConstPtr& msg)
    {
      double radius;
      double motor_voltage,motor_speed,wheel_speed,linear_speed;

      if(fabs(this->last_cmd_steer)>0.001)
        radius=fabs(this->axel_distance/tan(this->last_cmd_steer));
      else
        radius=std::numeric_limits<double>::max();

      /* convert input data [-1000 <-> 1000] */
      motor_voltage=(msg->data/4.0)*5.0/255.0;// cmd*pwm_max_voltage/pwm_number of steps
      motor_speed=motor_voltage*1000.0; // motor_voltage*speed_controller conversion factor (1V <-> 1000 rev/min)
      wheel_speed=motor_speed/5.5; // motor_speed / gear_ratio of the car
      linear_speed=wheel_speed*3.13159*this->wheel_diameter/60.0; // linear speed in m/s

      ROS_DEBUG("SeatCarController: Speed=%d, MotorVoltage=%f, MotorSpeed=%f, WheelSpeed=%f, LinearSpeed=%f", msg->data, motor_voltage, motor_speed, wheel_speed, linear_speed);
      //std::cout << msg->data << "," << motor_voltage << "," << motor_speed << "," << wheel_speed << "," << linear_speed << std::endl;

      if(this->last_cmd_steer>0)
      {
        // right inner
        this->rear_right_cmd=(linear_speed*((radius-this->wheel_distance/2.0)/(radius*this->wheel_diameter/2.0)));
        // left outer
        this->rear_left_cmd=-(linear_speed*((radius+this->wheel_distance/2.0)/(radius*this->wheel_diameter/2.0)));
      }
      else
      {
        // right outer
        this->rear_right_cmd=(linear_speed*((radius+this->wheel_distance/2.0)/(radius*this->wheel_diameter/2.0)));
        // left inner
        this->rear_left_cmd=-(linear_speed*((radius-this->wheel_distance/2.0)/(radius*this->wheel_diameter/2.0)));
      }

      this->last_cmd_drive=linear_speed;
    }

  template <class HardwareInterface>
    void SeatCarController<HardwareInterface>::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
    {
      std_msgs::Float32 yaw_msg;
 
      this->heading=tf::getYaw(msg->orientation);
      yaw_msg.data=-this->heading; //*180.0/3.14159;
      this->yaw_pub.publish(yaw_msg); 
    }

} // namespace

#endif // header guard
