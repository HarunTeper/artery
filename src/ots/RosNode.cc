/*
 * Simulation.cpp
 *
 *  Created on: 18. 11. 2015
 *      Author: Vladimir Matena
 */

#include "RosNode.h"

#include <string>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

RosNode::RosNode() {
	rclcpp::init(0,nullptr);
	nodePtr = rclcpp::Node::make_shared("");
}

RosNode::RosNode(std::string nodeName) {
	rclcpp::init(0,nullptr);
	nodePtr = rclcpp::Node::make_shared(nodeName);
}

RosNode::~RosNode() {
}

rclcpp::Node::SharedPtr RosNode::getRosNode() {
	return nodePtr;
}

void RosNode::runNode(){
	rosThread = new std::thread([this](){rclcpp::spin(nodePtr);});
}

void RosNode::stopNode() {
	rclcpp::shutdown();
	rosThread->join();
}

