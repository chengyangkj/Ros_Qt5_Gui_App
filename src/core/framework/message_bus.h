#pragma once
#include <functional>
#include <map>
#include <mutex>
#include <vector>
#include <memory>
#include <string>
#include <atomic>
#include <type_traits>
#include <typeinfo>
#include "logger/logger.h"

#include "callback_executor.h"

namespace Framework {
class MessageBus;
}

// 类型特征：提取函数/可调用对象的参数类型
namespace Framework {
namespace detail {
  template<typename T>
  struct function_traits;

  // std::function 特化
  template<typename R, typename Arg>
  struct function_traits<std::function<R(Arg)>> {
    using arg_type = std::decay_t<Arg>;
  };

  // 函数指针特化
  template<typename R, typename Arg>
  struct function_traits<R(*)(Arg)> {
    using arg_type = std::decay_t<Arg>;
  };

  // 成员函数指针特化 (const)
  template<typename T, typename R, typename Arg>
  struct function_traits<R(T::*)(Arg) const> {
    using arg_type = std::decay_t<Arg>;
  };

  // 成员函数指针特化 (非const)
  template<typename T, typename R, typename Arg>
  struct function_traits<R(T::*)(Arg)> {
    using arg_type = std::decay_t<Arg>;
  };

  // lambda 和其他可调用对象：通过 operator() 提取
  template<typename T>
  struct function_traits : function_traits<decltype(&T::operator())> {};

  // 提取 lambda 的参数类型
  template<typename Lambda>
  struct lambda_traits {
    using arg_type = typename function_traits<Lambda>::arg_type;
  };
}
}

#ifdef _WIN32
#define FRAMEWORK_EXPORT __declspec(dllexport)
#else
#define FRAMEWORK_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {
FRAMEWORK_EXPORT Framework::MessageBus* GetMessageBusInstance();
}

namespace Framework {

// 类型擦除回调包装器基类
class CallbackBase {
 public:
  virtual ~CallbackBase() = default;
  virtual void call(const void* data, const std::type_info& type) = 0;
  virtual const std::type_info& getType() const = 0;
};

// 类型安全的回调包装器
template<typename T>
class TypedCallback : public CallbackBase {
 public:
  explicit TypedCallback(std::function<void(const T&)> callback)
      : callback_(callback) {}
  
  void call(const void* data, const std::type_info& type) override {
    // 类型匹配检查：如果发布者类型与订阅者类型不匹配，则不调用
    if (typeid(T) == type) {
      callback_(*static_cast<const T*>(data));
    } else {
      LOG_WARN("[TypedCallback::call] Type mismatch: expected " 
                << typeid(T).name() << ", got " << type.name());
    }
  }
  
  const std::type_info& getType() const override {
    return typeid(T);
  }

 private:
  std::function<void(const T&)> callback_;
};


class MessageBus {
 public:
  using CallbackId = size_t;
  
  static MessageBus& Instance();

  static MessageBus* GetInstance() {
    return &Instance();
  }
  
  template<typename T>
  void Publish(const std::string& topic, const T& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subscribers_.find(topic);
    if (it != subscribers_.end()) {
      const std::type_info& data_type = typeid(T);
      size_t subscriber_count = it->second.size();
      // LOG_INFO("[MessageBus::Publish] topic: " << topic 
      //           << ", type: " << data_type.name() 
      //           << ", subscribers: " << subscriber_count);
      for (auto& pair : it->second) {
        if (pair.second) {
          // 保存数据的副本（因为可能在异步调用中使用）
          auto callback_ptr = pair.second.get();
          auto data_copy = std::make_shared<T>(data);
          // typeid(T) 返回的引用在整个程序生命周期内有效，可以直接使用
          const std::type_info* type_ptr = &typeid(T);
          detail::ThreadSafeCallbackExecutor::Execute([callback_ptr, data_copy, type_ptr]() {
            // TypedCallback 内部会进行类型匹配检查
            // LOG_INFO("[MessageBus::Publish] Executing callback, type: " << type_ptr->name());
            callback_ptr->call(static_cast<const void*>(data_copy.get()), *type_ptr);
          });
        }
      }
    } else {
      LOG_INFO("[MessageBus::Publish] topic: " << topic 
                << ", type: " << typeid(T).name() 
                << ", no subscribers");
    }
  }
  
  template<typename T>
  CallbackId Subscribe(const std::string& topic, std::function<void(const T&)> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    CallbackId id = next_callback_id_.fetch_add(1);
    // 直接存储类型 T 的回调，无需 std::any 转换
    subscribers_[topic][id] = std::make_unique<TypedCallback<T>>(callback);
    LOG_INFO("[MessageBus::Subscribe] topic: " << topic 
              << ", type: " << typeid(T).name() 
              << ", callback_id: " << id);
    return id;
  }
  
  void Unsubscribe(const std::string& topic, CallbackId id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subscribers_.find(topic);
    if (it != subscribers_.end()) {
      it->second.erase(id);
      if (it->second.empty()) {
        subscribers_.erase(it);
      }
    }
  }
  
  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    subscribers_.clear();
  }

 private:
  MessageBus() : next_callback_id_(1) {}
  ~MessageBus() = default;
  MessageBus(const MessageBus&) = delete;
  MessageBus& operator=(const MessageBus&) = delete;
  
  mutable std::mutex mutex_;
  std::map<std::string, std::map<CallbackId, std::unique_ptr<CallbackBase>>> subscribers_;
  std::atomic<CallbackId> next_callback_id_{1};
};

}  // namespace Framework

