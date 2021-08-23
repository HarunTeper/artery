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

RosNode RosNode::instance;

RosNode::RosNode() {
	rclcpp::init(0,nullptr);
	nodePtr = rclcpp::Node::make_shared("rosNode");
	// rosThread = new std::thread([this](){rclcpp::spin(nodePtr);});
}

RosNode::RosNode(std::string nodeName) {
	// const char* args[1] = { nodeName.c_str() }; 
	// rclcpp::init(1,args);
	rclcpp::init(0,nullptr);
	nodePtr = rclcpp::Node::make_shared(nodeName);
	// rosThread = new std::thread([this](){rclcpp::spin(nodePtr);});
}

RosNode::~RosNode() {
}

RosNode& RosNode::getInstance() {
	return instance;
}

rclcpp::Node::SharedPtr RosNode::getRosNode() {
	return nodePtr;
}

void RosNode::spin_some(){
	rclcpp::spin_some(this->getRosNode());
}

void RosNode::runNode(){
}

void RosNode::stopNode() {
	rclcpp::shutdown();
	rosThread->join();
}

