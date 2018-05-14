/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_LIGHT_PLUGIN_HH_
#define _GAZEBO_LIGHT_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Color.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>

//namespace gazebo
//{
  class ColorPlugin : public gazebo::VisualPlugin
  {
    public:
      
      ColorPlugin();
      virtual  ~ColorPlugin();
      void Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate();
    private:
      boost::thread pluginThread;
      void threadFunction();

      gazebo::rendering::VisualPtr model;
      gazebo::event::ConnectionPtr updateConnection;

      ros::NodeHandle* nh;
      ros::Subscriber material_subscriber;
      void materialCallback(const std_msgs::String::ConstPtr& msg);
      bool new_material;
      ros::Subscriber color_subscriber;
      void colorCallback(const std_msgs::ColorRGBA::ConstPtr& msg);
      bool new_color;
      
      ros::ServiceServer trigger_server;
      bool triggerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      
      std::string plugin_name;
      std::string element_name;
      std::string material_name;
      std::string material1_name;
      std::string material2_name;
      bool trigger_state;
      gazebo::common::Color color;
  };
//}

#endif 
