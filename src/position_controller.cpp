#include <fetch_giskard/position_controller.h>

#include <robot_controllers_interface/controller_manager.h>

#include <sstream>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::PositionController, robot_controllers::Controller)

namespace robot_controllers {

int PositionController::init(ros::NodeHandle& nh, ControllerManager* manager) {
  Controller::init(nh, manager);

  std::string paramCJ;
  nh.getParam("controlled_joints", paramCJ);
  std::stringstream jointStream(paramCJ);

  XmlRpc::XmlRpcValue names;
  if (!nh.getParam("joints", names)) {
    ROS_ERROR_STREAM("No joints given for " << nh.getNamespace());
    return -1;
  }

  if (names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("Joints not in a list for " << nh.getNamespace());
    return -1;
  }

  for (int i = 0; i < names.size(); ++i) {
    XmlRpc::XmlRpcValue &name_value = names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR_STREAM("Not all joint names are strings for " << nh.getNamespace());
      return -1;
    }

    std::string jname = static_cast<std::string>(name_value);
    JointHandlePtr joint = manager->getJointHandle(jname);
    if (!joint) {
      ROS_ERROR_NAMED("JointPositionController", "Can't get handle for joint '%s' from manager.", jname.c_str());
      return -1;
    }

    // Effort is constant for now
    controlledJoints[jname] = {joint, ros::Time(0), 0.0};
  }

  this->nh = nh;

  ROS_INFO("Position controller successfully initialized");

  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool PositionController::start() {
  cmdSubscriber = nh.subscribe("commands", 10, &PositionController::commandCallback, this);
  ROS_INFO("Position controller started");
  return true;
}

bool PositionController::stop(bool force) {
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++) {
    it->second.desiredPosition = it->second.joint->getPosition();
    it->second.joint->setPosition(it->second.desiredPosition, 0, 0);
  }
  ROS_INFO("Position controller stopped");
  return true;
}

bool PositionController::reset() {
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++)
    stopJoint(it->second);
  return true;
}

void PositionController::update(const ros::Time& time, const ros::Duration& dt) {
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++) {
    it->second.joint->setPosition(it->second.desiredPosition, 0, 0);
  }
}

std::vector<std::string> PositionController::getCommandedNames() {
  std::vector<std::string> out;
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++)
    out.push_back(it->first);
  return out;
}

void PositionController::commandCallback(const sensor_msgs::JointState& command) {
  if (command.position.size() != command.name.size()) {
    ROS_ERROR_NAMED("JointPositionController", "Malformed command message. Position and name list must be of equal length.");
    return;
  }

  if (command.effort.size() != 0 && command.effort.size() != command.name.size()) {
     ROS_ERROR_NAMED("JointPositionController", "Malformed command message. Effort list must either be empty or have a number of entries matching the length of the name list.");
     return;
  }

  for (size_t i = 0; i < command.name.size(); i++) {
    const std::string& name = command.name[i];
    auto it = controlledJoints.find(name);

    if (it != controlledJoints.end()) {
        it->second.desiredPosition = command.position[i];
    } else {
      ROS_ERROR_NAMED("JointPositionController", "Can't process command for joint '%s' as it's not commanded by this controller.", command.name[i].c_str());
    }
  }
}

void PositionController::stopJoint(ControlledJoint& c) {
  c.desiredPosition = c.joint->getPosition();
  c.joint->setPosition(c.desiredPosition, 0, 0);
}

}  // namespace robot_controllers
