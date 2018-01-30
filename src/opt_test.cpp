#include <ros/ros.h>
#include "giskard_suturo_parser/parser.h"
#include <giskard_core/giskard_core.hpp>
#include <giskard_suturo_parser/utils.h>
#include <iai_naive_kinematics_sim/SetJointState.h>

#include <sensor_msgs/JointState.h>

#include <map>
#include <stdlib.h>

using namespace giskard_suturo;
using namespace giskard_core;
using namespace std;
using namespace ros;


int main(int argc, char *argv[])
{
    if (argc < 2) {
        cerr << "Needed <gpp file> [<iterations>]" << endl;
        return 0;
    }


    int iterations = 500;
    if (argc > 2) {
        iterations = atoi(argv[2]);
    }

    ros::init(argc, argv, "gnp_opt");
    NodeHandle nh = NodeHandle();

	GiskardPPParser parser;

    QPControllerSpec qpSpec = parser.parseFromFile(argv[1]);
    QPController controller = generate(qpSpec);

    map<string, double> js;
    auto jointInputs = controller.get_input_map(tJoint);
    Eigen::VectorXd state = Eigen::VectorXd::Zero(controller.get_input_size());

    for (auto it = jointInputs.begin(); it != jointInputs.end(); it++)
        js[it->first] = 0;

    cout << "Starting controller with " << js.size() << " controllables..." << endl;
    if (controller.start(state, 1000)) {
        cout << "Running" << endl;
        int i = 0;
        for (; i < iterations && controller.update(state, 1000); i++) {
            auto newCmd = controller.get_command_map();
            for (auto it = newCmd.begin(); it != newCmd.end(); it++) {
                js[it->first] += it->second * 0.25;
                controller.set_input(state, it->first, js[it->first]);
            }
        }
        cout << "Execution finished. " << i << " cycles were performed." << endl;
    }

    sensor_msgs::JointState final_state_t0, final_state_t1;
    final_state_t0.header.stamp = Time::now();
    final_state_t1.header.stamp = Time::now();

    auto state_t0_srv = nh.serviceClient<iai_naive_kinematics_sim::SetJointState>("/simulator_t0/set_joint_states");
    auto state_t1_srv = nh.serviceClient<iai_naive_kinematics_sim::SetJointState>("/simulator_t1/set_joint_states");

    //ros::spinOnce();

    for (auto it = js.begin(); it != js.end(); it++) {
        if (it->first.find("t0_") == 0) {
            final_state_t0.name.push_back(it->first.substr(3));
            final_state_t0.position.push_back(it->second);
            final_state_t0.velocity.push_back(0);
            final_state_t0.effort.push_back(0);
            cout << "Position of " << it->first << " is " << it->second << endl;
        } else if (it->first.find("t1_") == 0) {
            final_state_t1.name.push_back(it->first.substr(3));
            final_state_t1.position.push_back(it->second);
            final_state_t1.velocity.push_back(0);
            final_state_t1.effort.push_back(0);
            cout << "Position of " << it->first << " is " << it->second << endl;
        }
    }

    iai_naive_kinematics_sim::SetJointState call_t0, call_t1;
    call_t0.request.state = final_state_t0;
    call_t1.request.state = final_state_t1;

    if (!state_t0_srv.call(call_t0))
        cerr << "Call to set joints in simulator_t0 failed!" << endl;

    if (!state_t1_srv.call(call_t1))
        cerr << "Call to set joints in simulator_t1 failed!" << endl;

	return 0;
}