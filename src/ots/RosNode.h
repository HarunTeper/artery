#ifndef SIMULATION_H
#define SIMULATION_H

#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class RosNode {
public:
	RosNode();
	RosNode(std::string name);
	~RosNode();

	rclcpp::Node::SharedPtr getRosNode() ;

	void runNode();
	void stopNode();

private:
	std::string name;
	rclcpp::Node::SharedPtr nodePtr;
	std::thread* rosThread;

	std::vector<rclcpp::Subscription<void*>::SharedPtr> nodeSubscribers;
	std::vector<rclcpp::Publisher<void*>::SharedPtr> nodePublishers;
};

#endif /* SIMULATION_H */
