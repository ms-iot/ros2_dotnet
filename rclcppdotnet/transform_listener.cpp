#include <string>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "transform_listener.h"



void* native_construct_buffer() {
	auto clock = std::make_shared<rclcpp::Clock>();
	return new tf2_ros::Buffer(clock);
}

void* native_construct_listener(void* buf) {
	return new tf2_ros::TransformListener(*((tf2_ros::Buffer*)buf));
}

void* native_construct_time(int sec, int nano) {
	return new rclcpp::Time(sec, nano);
}

void* native_lookup_transform(void* buf,
	char* from, char* to, void* t) {
	auto outputVar = new geometry_msgs::msg::TransformStamped(
		((tf2_ros::Buffer*)buf)->lookupTransform((char*)from, (char*)to, *((rclcpp::Time*)t))
	);
	return outputVar;
}

bool native_retrieve_translation(void* tf, TfVector3Ptr vec) {
	try {
		auto castedTf = (geometry_msgs::msg::TransformStamped*)tf;
		vec->x = tf->transform.translation.x;
		vec->y = tf->transform.translation.y;
		vec->z = tf->transform.translation.z;
		return true;
	}
	catch (...) {
		return false;
	}
	return false;
}

bool native_retrieve_rotation(void* tf, TfQuaternionPtr quat) {
	try {
		auto castedTf = (geometry_msgs::msg::TransformStamped*)tf;
		quat->x = tf->transform.rotation.x;
		quat->y = tf->transform.rotation.y;
		quat->z = tf->transform.rotation.z;
		quat->w = tf->transform.rotation.w;
		return true;
	}
	catch (...) {
		return false;
	}
	return false;
}