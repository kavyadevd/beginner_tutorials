#include <ros/service_client.h>
#include "beginner_tutorials/ServiceFile.h"

std::shared_ptr<ros::NodeHandle> nh;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
* @brief Method to check if publishing node exists.
*/
TEST(TestSimulatorStub, TestPublisherExists) {
    ros::Subscriber test_subscriber = nh.subscribe("talker", 1000, chatterCallback);
    EXPECT_TRUE(exists(test_subscriber));
}
