#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "gazebo_object_attacher.h"
#include "gazebo_object_attacher/Attach.h"
#include "gazebo_object_attacher/AttachRequest.h"
#include "gazebo_object_attacher/AttachResponse.h"

using namespace std;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(gazebo_attacher)

  // Constructor
  gazebo_attacher::gazebo_attacher() :
    nh_("")
  {
  }


  // Destructor
  gazebo_attacher::~gazebo_attacher() {
  }

  void gazebo_attacher::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) {

    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    
		world = _world;
    physics = this->world->GetPhysicsEngine();
    attach_service_ = this->nh_.advertiseService("/attach_object", &gazebo_attacher::attach_callback, this);
    detach_service_ = this->nh_.advertiseService("/detach_object", &gazebo_attacher::detach_callback, this);
  }

  bool gazebo_attacher::attach(string robot_name, std::string robot_link_name, string object_name) {


    physics::ModelPtr robot_model = world->GetModel(robot_name);

		if( !robot_model ) {
			ROS_ERROR_STREAM("Cannot find robot model!");	
			return false;	
		}

		_Link = robot_model->GetLink(robot_link_name);

		if( !_Link ) {
			ROS_ERROR_STREAM("Cannot find robot link!");	
			return false;	
		}

 		physics::ModelPtr obj_model = world->GetModel(object_name);

		if( !obj_model ) {
			ROS_ERROR_STREAM("Cannot find object!");	
			return false;	
		}
		
		gazebo::math::Pose diff = obj_model->GetLink()->GetWorldPose() - _Link->GetWorldPose();
 	

 		_fixedJoint = physics->CreateJoint("fixed");
		_fixedJoint->Load(_Link,obj_model->GetLink(), diff);
    _fixedJoint->Init();
    _fixedJoint->SetHighStop(0, 0);
		_fixedJoint->SetLowStop(0, 0);

    return true;
  }

  bool gazebo_attacher::detach(string robot_name, std::string robot_link_name, string object_name) {

    physics::ModelPtr robot_model = world->GetModel(robot_name);

		if( !robot_model ) {
			ROS_ERROR_STREAM("Cannot find robot model!");	
			return false;	
		}

		_Link = robot_model->GetLink(robot_link_name);

		if( !_Link ) {
			ROS_ERROR_STREAM("Cannot find robot link!");	
			return false;	
		}

 		physics::ModelPtr obj_model = world->GetModel(object_name);

		if( !obj_model ) {
			ROS_ERROR_STREAM("Cannot find object!");	
			return false;	
		}
		
   	_fixedJoint->Detach();



    return true;
  }


  bool gazebo_attacher::attach_callback(gazebo_object_attacher::Attach::Request &req,
                                              gazebo_object_attacher::Attach::Response &res) {

    ROS_INFO_STREAM("Received request to attach model: '" << req.robot_name << "' using link: '" << req.robot_link_name << "' with model: '" << req.object_name << "'");
    if (! this->attach(req.robot_name, req.robot_link_name,
                       req.object_name )){
      ROS_ERROR_STREAM("Could not make the attach.");
      res.ok = false;
    }
    else{
      ROS_INFO_STREAM("Attach was succesful");
      res.ok = true;
    }


    return true;

  }

  bool gazebo_attacher::detach_callback(gazebo_object_attacher::Attach::Request &req,
                                              gazebo_object_attacher::Attach::Response &res){
   ROS_INFO_STREAM("Received request to detach model: '" << req.robot_name << "' using link: '" << req.robot_link_name << "' with model: '" << req.object_name << "'");
    if (! this->detach(req.robot_name, req.robot_link_name,
                       req.object_name )){
      ROS_ERROR_STREAM("Could not make the detach.");
      res.ok = false;
    }
    else{
      ROS_INFO_STREAM("Attach was succesful");
      res.ok = true;
    }

      return true;
  }

}
