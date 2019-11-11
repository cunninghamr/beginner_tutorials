/**
 * @file set_msg_service_test.cpp
 * @brief Unit test for service that sets message of talker node.
 * @author Ryan Cunningham
 * @copyright 2019
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include <memory>

#include "beginner_tutorials/SetMsg.h"

TEST(SetMsgServiceTest, testSetMessageSuccess) {
  ros::NodeHandle nh;

  /// create service client to interface service server
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::SetMsg>("set_msg");

  /// create request
  beginner_tutorials::SetMsg srv;
  srv.request.data = "New message.";

  /// call service with request
  if (client.call(srv)) {
    /// verify the call succeeded and the message is correct
    EXPECT_EQ(true, srv.response.success);
    EXPECT_EQ("Message successfully set.", srv.response.message);
  } else {
    /// service does not return false, so this should not happen
    FAIL() << "SetMsgService should not return false.";
  }
}

TEST(SetMsgServiceTest, testEmptyMessageWarning) {
  ros::NodeHandle nh;

  /// create service client to interface service server
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::SetMsg>("set_msg");

  /// create request
  beginner_tutorials::SetMsg srv;
  srv.request.data = "";

  /// call service with request
  if (client.call(srv)) {
    /// verify the call succeeded and the message warns that the message
    /// is empty
    EXPECT_EQ(true, srv.response.success);
    EXPECT_EQ("Message string is empty!", srv.response.message);
  } else {
    /// service does not return false, so this should not happen
    FAIL() << "SetMsgService should not return false.";
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "set_msg_service_test");

  return RUN_ALL_TESTS();
}

