/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2024-01-01
 * @Description: Framework 统一接口头文件
 * 提供类型安全的发布订阅接口，使用者只需包含此头文件即可
 */
#pragma once

#include "core/framework/message_bus.h"

namespace Framework {
// 接口类和宏定义见下方
}  // namespace Framework

/**
 * @brief 发布消息宏
 * @param topic 主题名称（字符串）
 * @param data 要发布的数据（任意类型）
 * 
 * 使用示例:
 *   PUBLISH(MSG_ID_ROBOT_POSE, pose);
 */
#define PUBLISH(topic, data) \
  GetMessageBusInstance()->Publish(topic, data)

/**
 * @brief 订阅消息宏（自动类型推导）
 * @param topic 主题名称（字符串）
 * @param callback 回调函数（lambda 或函数对象）
 * 
 * 自动根据回调函数的参数类型进行类型推导，确保类型安全
 * 
 * 使用示例:
 *   SUBSCRIBE(MSG_ID_ROBOT_POSE, [this](const RobotPose& pose) {
 *     // 处理数据
 *   });
 */
#define SUBSCRIBE(topic, callback) \
  [&]() { \
    auto&& cb = callback; \
    using CallbackType = std::decay_t<decltype(cb)>; \
    using ArgType = typename Framework::detail::lambda_traits<CallbackType>::arg_type; \
    return GetMessageBusInstance()->Subscribe<ArgType>(topic, std::function<void(const ArgType&)>(std::forward<decltype(cb)>(cb))); \
  }()

/**
 * @brief 取消订阅宏
 * @param topic 主题名称（字符串）
 * @param id 订阅时返回的 ID
 * 
 * 使用示例:
 *   auto id = SUBSCRIBE(MSG_ID_ROBOT_POSE, callback);
 *   UNSUBSCRIBE(MSG_ID_ROBOT_POSE, id);
 */
#define UNSUBSCRIBE(topic, id) \
  GetMessageBusInstance()->Unsubscribe(topic, id)

