#ifndef SIMPLE_STATE_CONTROLLER_H
#define SIMPLE_STATE_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>

namespace my_simple_controllers {

  class StateController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
    public:
      //method executed by controller manager on every loop when controller is running
      //NOTE: MUST BE implemented
      virtual void update(const ros::Time& time, const ros::Duration& period);

      //initialize controller with access to hardware interface and node handles
      virtual bool init(hardware_interface::JointStateInterface* hw, 
		      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  };


};

#endif
