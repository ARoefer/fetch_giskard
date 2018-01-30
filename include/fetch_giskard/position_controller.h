#pragma once

#include <ros/ros.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/joint_handle.h>
#include <sensor_msgs/JointState.h>

#include <unordered_map>
#include <vector>
#include <string>

namespace robot_controllers {

/**
 * @brief Controller class to control robot using joint velocity goals.
 */
class PositionController : public Controller {
  struct ControlledJoint {
    JointHandlePtr joint;
    ros::Time lastCommandStamp;
    double desiredPosition;
  };

public:
  /**
   * @brief Default constructor, does almost nothing, all setup is done in init().
   */
  PositionController() {
    ROS_INFO("Position controller created");
  }

  /**
   * @brief Initialize the controller by reading the names of the controlled joints from the parameter server.
   * @param nh Node handle for this controller.
   * @param manager The controller manager instance, this is needed for the
   *        controller to get information about joints, etc.
   * @returns 0 if succesfully configured, negative values are error codes.
   */
  virtual int init(ros::NodeHandle& nh, ControllerManager* manager);

  /**
   * @brief Attempt to start the controller by subscribing to the "joint_velocity_commands" topic.
   * @returns Always true.
   */
  virtual bool start();

  /**
   * @brief Attempt to stop the controller. Stops all joint movements.
   * @param force Should we force the controller to stop? Some controllers
   *        may wish to continue running until they absolutely have to stop.
   * @returns Always true;
   */
  virtual bool stop(bool force);

  /**
   * @brief Cleanly reset the controller to it's initial state. Sets the desired velocity for all joints to zero and stops their motions.
   * @returns True if successfully reset, false otherwise.
   */
  virtual bool reset();

  /**
   * @brief This is the update loop for the controller. It applies the current set of commands to the joints, as long as they're not older than the watchdog-period.
   * @param time The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const ros::Time& time, const ros::Duration& dt);

  /** @brief Get the type of this controller. */
  virtual std::string getType() {
    return "JointPositionController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames() { return std::vector<std::string>(); }

  /** @brief Callback for refreshing joint commands. The stamp of the command needs to be set correctly */
  virtual void commandCallback(const sensor_msgs::JointState& command);

private:
  /** @brief Internal function for stopping a joint */
  void stopJoint(ControlledJoint& c);

  /** @brief Subscriber for commands */
  ros::Subscriber cmdSubscriber;

  /** @brief Node handle of manager */
  ros::NodeHandle nh;

  /** @brief Map of joints controlled by this node */
  std::unordered_map<std::string, ControlledJoint> controlledJoints;
};

}  // namespace robot_controllers
