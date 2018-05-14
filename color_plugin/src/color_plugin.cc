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
#include "color_plugin.h"

//using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ColorPlugin)

ColorPlugin::ColorPlugin()
{
  this->nh = NULL;
  this->material_name="Gazebo/RedGlow";
  this->material1_name="Gazebo/RedGlow";
  this->material2_name="Gazebo/GreenGlow";
  this->plugin_name="empty_plugin_name";
  this->color.Set(1, 0 , 0);
  this->new_material=false;
  this->new_color=false;
  this->trigger_state=false;
  
}

ColorPlugin::~ColorPlugin()
{
  delete this->nh;
}

void ColorPlugin::Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;
  
  this->plugin_name = this->GetHandle().c_str();

  if(_sdf->GetElement("sdf")->HasElement("material"))
  {
    this->material1_name =  _sdf->GetElement("sdf")->Get<std::string>("material");
    this->material_name= this->material1_name;
    this->new_material=true;
  }
  
  if(_sdf->GetElement("sdf")->HasElement("material2"))
  {
    this->material2_name =  _sdf->GetElement("sdf")->Get<std::string>("material2");
    this->new_material=true;
  }

  this->pluginThread = boost::thread(boost::bind(&ColorPlugin::threadFunction, this));
  this->updateConnection = gazebo::event::Events::ConnectPreRender(boost::bind(&ColorPlugin::OnUpdate, this));
}

void ColorPlugin::threadFunction()
{
  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  else
  {
    this->nh = new ros::NodeHandle("~");
    std::string material_topic = "/" + this->plugin_name + "/" + "set_material";
    std::string color_topic = "/" + this->plugin_name + "/" + "set_color";
    std::string srv_topic = "/" + this->plugin_name + "/" + "trigger";
    this->material_subscriber = this->nh->subscribe<std_msgs::String>(material_topic, 1, boost::bind(&ColorPlugin::materialCallback, this, _1));
    this->color_subscriber    = this->nh->subscribe<std_msgs::ColorRGBA>(color_topic, 1, boost::bind(&ColorPlugin::colorCallback, this, _1));
    this->trigger_server      = this->nh->advertiseService(srv_topic, &ColorPlugin::triggerCallback, this);
  }
}

void ColorPlugin::materialCallback(const std_msgs::String::ConstPtr& _msg) 
{
  std_msgs::String name_msg = *_msg;

  if(!this->trigger_state)
    this->material1_name=name_msg.data;
  else
    this->material2_name=name_msg.data;
  
  this->material_name = name_msg.data;
  this->new_material=true;
}

void ColorPlugin::colorCallback(const std_msgs::ColorRGBA::ConstPtr& _msg) 
{  
  this->color.Set(_msg->r, _msg->g, _msg->b);
  this->new_color=true;
}

bool ColorPlugin::triggerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if(!this->trigger_state)
    this->material_name=this->material2_name;
  else
    this->material_name=this->material1_name;
  this->trigger_state = !this->trigger_state;
  this->new_material=true;
  return true;
}

void ColorPlugin::OnUpdate()
{
  if(this->new_material)
  {
    this->model->SetMaterial(this->material_name);
    this->new_material=false;
  }
  else if(this->new_color)
  {
    this->model->SetAmbient(this->color);
    this->model->SetDiffuse(this->color);
    this->model->SetEmissive(this->color);
    this->new_color=false;
  }
}