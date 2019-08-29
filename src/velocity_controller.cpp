#include <fetch_giskard/velocity_controller.h>

#include <robot_controllers_interface/controller_manager.h>

#include <sstream>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::VelocityController, robot_controllers::Controller)

namespace robot_controllers {

int VelocityController::init(ros::NodeHandle& nh, ControllerManager* manager) {
  Controller::init(nh, manager);

  nh.getParam("watchdog_period", watchdogPeriod);

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
      ROS_ERROR_NAMED("JointVelocityController", "Can't get handle for joint '%s' from manager.", jname.c_str());
      return -1;
    }

    // Effort is constant for now
    controlledJoints[jname] = {joint, ros::Time(0), 0.0};
  }

  this->nh = nh;

  ROS_INFO("Velocity controller successfully initialized");

  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool VelocityController::start() {
  cmdSubscriber = nh.subscribe("commands", 10, &VelocityController::commandCallback, this);
  ROS_INFO("Velocity controller started");
  return true;
}

bool VelocityController::stop(bool force) {
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++) {
    it->second.joint->setVelocity(0, 0);
    it->second.desiredVelocity = 0;
  }
  ROS_INFO("Velocity controller stopped");
  return true;
}

bool VelocityController::reset() {
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++)
    stopJoint(it->second);
  return true;
}

void VelocityController::update(const ros::Time& time, const ros::Duration& dt) {
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++) {
    if ((time - it->second.lastCommandStamp).toSec() <= watchdogPeriod) { // || it->second.lastCommandStamp == ros::Time(0)) {

      it->second.joint->setVelocity(it->second.desiredVelocity, 0);
    } else if (it->second.lastCommandStamp != ros::Time(0)) {
      std::cout << "Command is older than watchdog period. Stopping joint " << it->first << std::endl
                << " Time: " << time.toSec() << std::endl
                << "Stamp: " << it->second.lastCommandStamp.toSec() << std::endl
                << "Difference: " << (time - it->second.lastCommandStamp).toSec() << std::endl;
      stopJoint(it->second);
    }
  }
}

std::vector<std::string> VelocityController::getCommandedNames() {
  std::vector<std::string> out;
  for (auto it = controlledJoints.begin(); it != controlledJoints.end(); it++)
    out.push_back(it->first);
  return out;
}

void VelocityController::commandCallback(const sensor_msgs::JointState& command) {
  if (command.velocity.size() != command.name.size()) {
    ROS_ERROR_NAMED("JointVelocityController", "Malformed command message. Velocity and name list must be of equal length.");
    return;
  }

  if (command.effort.size() != command.name.size()) {
     ROS_ERROR_NAMED("JointVelocityController", "Malformed command message. Effort list must either be empty or have a number of entries matching the length of the name list.");
     return;
  }

  for (size_t i = 0; i < command.name.size(); i++) {
    const std::string& name = command.name[i];
    auto it = controlledJoints.find(name);

    if (it != controlledJoints.end()) {
      // Only consider commands that are newer
      if (it->second.lastCommandStamp < command.header.stamp) {
        double newVelocity = command.velocity[i];
        //if (abs(newVelocity) > it->second.joint->getVelocityMax())
        //  newVelocity = copysign(it->second.joint->getVelocityMax(), newVelocity);

        it->second.desiredVelocity = newVelocity;
        it->second.lastCommandStamp = command.header.stamp;
      }
    } else {
      ROS_ERROR_NAMED("JointVelocityController", "Can't process command for joint '%s' as it's not commanded by this controller.", command.name[i].c_str());
    }
  }
}

void VelocityController::stopJoint(ControlledJoint& c) {
  c.desiredVelocity = 0;
  c.lastCommandStamp = ros::Time(0);
  c.joint->setVelocity(0, 0);
}

}  // namespace robot_controllers
