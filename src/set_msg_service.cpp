/**                                                                                               
 * @file set_msg_service.cpp
 * @brief Service server to set the message of a message publishing node.
 * @author Ryan Cunningham
 * @copyright 2019
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 */ 

#include "set_msg_service.hpp"

#include <ros/console.h>

SetMsgService::SetMsgService(const std::string& defaultMsg): msg(defaultMsg) {}

bool SetMsgService::setMsg(beginner_tutorials::SetMsg::Request& req,
                           beginner_tutorials::SetMsg::Response& resp) {
  msg = req.data;
  resp.success = true;

  if (req.data.empty()) {
    resp.message = "Message string is empty!";

    ROS_WARN_STREAM("Message string is empty!");
  } else {
    resp.message = "Message successfully set.";

    ROS_INFO_STREAM("Message successfully set.");
  }

  return true;
}

std::string SetMsgService::getMsg() {
  return msg;
}
