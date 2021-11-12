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

/**
 * @brief Tests whether the changeBaseString service can modify the output message
 */
TEST(TestSimulatorStub, TestMessageChangeService) {
  auto client = nh.serviceClient
  <beginner_tutorials::ServiceFile>("ServiceFile");
  beginner_tutorials::ServiceFile service_object;
  service_object.request.inString = "Service test in progress";
  client.call(service_object.request, service_object.response);
  EXPECT_STREQ("Service test in progress", string(service_object.response.outString));
}
