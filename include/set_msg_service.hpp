/**
 * @file set_msg_service.hpp
 * @brief Service server to set the message of a message publishing node.
 * @author Ryan Cunningham
 * @copyright 2019
 * Distributed under the BSD License (license terms found in LICENSE or at https://www.freebsd.org/copyright/freebsd-license.html)
 */

#ifndef INCLUDE_SET_MSG_SERVICE_HPP_
#define INCLUDE_SET_MSG_SERVICE_HPP_

#include <string>

#include "beginner_tutorials/SetMsg.h"

class SetMsgService {
 public:
  /**
   * @brief Initialize service with default message.
   * @param defaultMsg
   */
  explicit SetMsgService(const std::string& defaultMsg);

  /**
   * @brief Sets the message string.
   * @param req
   * @param resp
   * @return true if message was set
   */
  bool setMsg(beginner_tutorials::SetMsg::Request& req,
              beginner_tutorials::SetMsg::Response& resp);

  /**
   * @brief Get the message string
   * @return the message string
   */
  std::string getMsg();

 private:
  /// the message string to alter
  std::string msg;
};

#endif  // INCLUDE_SET_MSG_SERVICE_HPP_
