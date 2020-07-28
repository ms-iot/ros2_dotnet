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

double native_retrieve_translation_x(void* tf) {
	return ((geometry_msgs::msg::TransformStamped*)tf)->transform.translation.x;
}

double native_retrieve_translation_y(void* tf) {
	return ((geometry_msgs::msg::TransformStamped*)tf)->transform.translation.y;
}

double native_retrieve_translation_z(void* tf) {
	return ((geometry_msgs::msg::TransformStamped*)tf)->transform.translation.z;
}