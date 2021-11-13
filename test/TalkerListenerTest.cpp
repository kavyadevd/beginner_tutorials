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
 * @file TalkerListenerTest.cpp
 * @author Kavyashree Devadiga
 * @date 13th November 2021
 * @copyright All rights reserved
 * @brief Cpp file to define test cases
 */

// Include required headers
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "std_msgs/String.h"

#include "beginner_tutorials/ServiceFile.h"

/**
 * @brief Test if service client is active
 */
TEST(TestStub, TestPublisherExists) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<beginner_tutorials::ServiceFile>("ServiceFile");
  bool exists(client.waitForExistence(ros::Duration(5.0)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Tests service to modify publisher message
 */
TEST(TestStub, TestMessageChangeService) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::ServiceFile>("ServiceFile");
  beginner_tutorials::ServiceFile::Request request_;
  beginner_tutorials::ServiceFile::Response response_;
  request_.input_msg = "Test Service message change";
  std::string expectedString = request_.input_msg;
  bool success = client.call(request_, response_);
  EXPECT_EQ("Test Service message change", response_.output_msg);
}
