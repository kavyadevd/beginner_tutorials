/*
 * Copyright (C) MIT.
 ** Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 *
 * @file TalkerListnerTest.cpp
 * @author Kavyashree Devadiga
 * @date 12th November 2021
 * @copyright All rights reserved
 * @brief File to define ROS test case methods.
 */

// Include required headers
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include "beginner_tutorials/ServiceFile.h"

std::shared_ptr<ros::NodeHandle> nh;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
* @brief Method to check if braodcaster node exists.
*/
TEST(TestSimulatorStub, TestBroadcasterExists) {
    ros::Subscriber test_subscriber = nh.subscribe("talker", 1000, chatterCallback);
    EXPECT_TRUE(exists(test_subscriber));
}
