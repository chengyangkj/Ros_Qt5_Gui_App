#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/client.hpp>

#include "rapidjson/document.h"

#include "itransport_layer.h"
#include "types.h"

using json = rapidjson::Document;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

namespace rosbridge2cpp{
  class SocketWebSocketConnection : public ITransportLayer{
    public:
      SocketWebSocketConnection() = default;
      
      ~SocketWebSocketConnection() {
        std::cout << "WebSocket Connection Destructor called" << std::endl;
        if (is_connected_) {
          Disconnect();
        }
        terminate_receiver_thread_ = true;
        if (receiver_thread_set_up_) {
          std::cout << "Thread is set up: Calling .join() on it" << std::endl;
          receiver_thread_.join();
          std::cout << "join() in Connection Destructor done" << std::endl;
        } else {
          std::cout << "receiverThread hasn't been set up. Skipping join() on it" << std::endl;
        }
      }

      bool Init(std::string p_ip_addr, int p_port);
      bool SendMessage(std::string data);
      int ReceiverThreadFunction();
      void RegisterIncomingMessageCallback(std::function<void(json&)> fun);
      void RegisterErrorCallback(std::function<void(TransportError)> fun);
      void ReportError(TransportError err);
      void SetTransportMode(ITransportLayer::TransportMode mode);
      void Disconnect();

    private:
      typedef websocketpp::client<websocketpp::config::asio> client;
      typedef websocketpp::connection_hdl connection_hdl;
      typedef websocketpp::config::asio::message_type::ptr message_ptr;
      
      std::string ip_addr_;
      int port_;
      std::string uri_;
      
      client c_;
      connection_hdl hdl_;
      websocketpp::lib::shared_ptr<websocketpp::lib::thread> asio_thread_;
      
      std::thread receiver_thread_;
      bool terminate_receiver_thread_ = false;
      bool receiver_thread_set_up_ = false;
      bool is_connected_ = false;
      bool callback_function_defined_ = false;
      
      std::function<void(json&)> incoming_message_callback_;
      std::function<void(TransportError)> error_callback_ = nullptr;
      
      std::mutex connection_mutex_;
      std::condition_variable connection_cv_;
      
      void on_open(connection_hdl hdl);
      void on_close(connection_hdl hdl);
      void on_fail(connection_hdl hdl);
      void on_message(connection_hdl hdl, message_ptr msg);
  };
}