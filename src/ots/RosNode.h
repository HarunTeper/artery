#ifndef SIMULATION_H
#define SIMULATION_H

#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <omnetpp/cobject.h>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class RosNode : omnetpp::cObject {
public:
	RosNode();
	RosNode(std::string name);
	~RosNode();

	static RosNode &getInstance();

	rclcpp::Node::SharedPtr getRosNode() ;

	void spin_some();
	void runNode();
	void stopNode();

private:
	static RosNode instance;

	std::string name;
	rclcpp::Node::SharedPtr nodePtr;
	std::thread* rosThread;
};

#endif /* SIMULATION_H */
