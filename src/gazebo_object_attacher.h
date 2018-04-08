/*
 * Desc: Gazebo link attacher plugin.
 * Author: Sammy Pfeiffer (sam.pfeiffer@pal-robotics.com)
 * Date: 05/04/2016
 */

#ifndef gazebo_object_attacher_HH
#define gazebo_object_attacher_HH

#include <ros/ros.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo_object_attacher/Attach.h"
#include "gazebo_object_attacher/AttachRequest.h"
#include "gazebo_object_attacher/AttachResponse.h"

using namespace std;

namespace gazebo
{

   class gazebo_attacher : public WorldPlugin
   {
      public:
        /// \brief Constructor
        gazebo_attacher();

        /// \brief Destructor
        virtual ~gazebo_attacher();

        /// \brief Load the controller
        void Load( physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/ );

        /// \brief Attach with a revolute joint
        bool attach(string robot_name, std::string robot_link_name, string object_name);

        /// \brief Detach
        bool detach(string robot_name, std::string robot_link_name, string object_name);

    
   private:
        ros::NodeHandle nh_;
        ros::ServiceServer attach_service_;
        ros::ServiceServer detach_service_;

        bool attach_callback(gazebo_object_attacher::Attach::Request &req,
                              gazebo_object_attacher::Attach::Response &res);
        bool detach_callback(gazebo_object_attacher::Attach::Request &req,
                             gazebo_object_attacher::Attach::Response &res);


        /// \brief The physics engine.
        physics::PhysicsEnginePtr physics;

        /// \brief Pointer to the world.
        physics::WorldPtr world;

 				physics::LinkPtr _Link;
				physics::JointPtr _fixedJoint;

   };

}

#endif

