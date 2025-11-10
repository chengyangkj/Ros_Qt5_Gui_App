#include "client/socket_websocket_connection.h"

namespace rosbridge2cpp{

  bool SocketWebSocketConnection::Init(std::string p_ip_addr, int p_port){
    ip_addr_ = p_ip_addr;
    port_ = p_port;
    
    // Construct WebSocket URI
    uri_ = "ws://" + ip_addr_ + ":" + std::to_string(port_);
    
    std::cout << "[WebSocketConnection] Initializing connection to " << uri_ << std::endl;
    
    try {
      // Set logging to be pretty verbose (everything except message payloads)
      c_.set_access_channels(websocketpp::log::alevel::all);
      c_.clear_access_channels(websocketpp::log::alevel::frame_payload);
      c_.set_error_channels(websocketpp::log::elevel::all);
      
      // Initialize ASIO
      c_.init_asio();
      
      // Register our message handler
      c_.set_message_handler(bind(&SocketWebSocketConnection::on_message, this, ::_1, ::_2));
      c_.set_open_handler(bind(&SocketWebSocketConnection::on_open, this, ::_1));
      c_.set_close_handler(bind(&SocketWebSocketConnection::on_close, this, ::_1));
      c_.set_fail_handler(bind(&SocketWebSocketConnection::on_fail, this, ::_1));
      
      // Create a connection to the given URI and queue it for connection once
      // the event loop starts
      websocketpp::lib::error_code ec;
      client::connection_ptr con = c_.get_connection(uri_, ec);
      if (ec) {
        std::cout << "[WebSocketConnection] Could not create connection because: " << ec.message() << std::endl;
        return false;
      }
      
      hdl_ = con->get_handle();
      c_.connect(con);
      
      // Start the ASIO io_service run loop
      asio_thread_ = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &c_);
      
      // Wait for connection to be established
      std::unique_lock<std::mutex> lock(connection_mutex_);
      if (!connection_cv_.wait_for(lock, std::chrono::seconds(5), [this] { return is_connected_; })) {
        std::cout << "[WebSocketConnection] Connection timeout" << std::endl;
        return false;
      }
      
      std::cout << "[WebSocketConnection] Connected successfully" << std::endl;
      
      // Setting up the receiver thread
      std::cout << "[WebSocketConnection] Setting up receiver thread..." << std::endl;
      receiver_thread_ = std::thread([=]() {ReceiverThreadFunction(); return 1; });
      receiver_thread_set_up_ = true;
      
      return true;
      
    } catch (websocketpp::exception const & e) {
      std::cout << "[WebSocketConnection] Exception: " << e.what() << std::endl;
      return false;
    }
  }

  bool SocketWebSocketConnection::SendMessage(std::string data){
    if (!is_connected_) {
      std::cout << "[WebSocketConnection] Not connected, cannot send message" << std::endl;
      return false;
    }
    
    try {
      websocketpp::lib::error_code ec;
      c_.send(hdl_, data, websocketpp::frame::opcode::text, ec);
      if (ec) {
        std::cout << "[WebSocketConnection] Send failed: " << ec.message() << std::endl;
        return false;
      }
      std::cout << "[WebSocketConnection] Data sent: " << data << std::endl;
      return true;
    } catch (websocketpp::exception const & e) {
      std::cout << "[WebSocketConnection] Send exception: " << e.what() << std::endl;
      return false;
    }
  }


  int SocketWebSocketConnection::ReceiverThreadFunction(){
    std::cout << "[WebSocketConnection] Receiver thread started" << std::endl;
    
    // The WebSocket client handles message reception in the on_message callback
    // This thread just waits for termination
    while (!terminate_receiver_thread_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "[WebSocketConnection] Receiver thread terminated" << std::endl;
    return 0;
  }

  void SocketWebSocketConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun){
    incoming_message_callback_ = fun;
    callback_function_defined_ = true;
  }

  void SocketWebSocketConnection::RegisterErrorCallback(std::function<void(TransportError)> fun){
    error_callback_ = fun;
  }

  void SocketWebSocketConnection::ReportError(TransportError err){
    if (error_callback_ == nullptr)
      return;
    error_callback_(err);
  }

  void SocketWebSocketConnection::SetTransportMode(ITransportLayer::TransportMode mode){
    // Only JSON mode is supported
  }

  void SocketWebSocketConnection::Disconnect(){
    if (!is_connected_) {
      return;
    }
    
    try {
      websocketpp::lib::error_code ec;
      c_.close(hdl_, websocketpp::close::status::normal, "", ec);
      if (ec) {
        std::cout << "[WebSocketConnection] Error on close: " << ec.message() << std::endl;
      }
    } catch (websocketpp::exception const & e) {
      std::cout << "[WebSocketConnection] Exception on close: " << e.what() << std::endl;
    }
    
    is_connected_ = false;
    terminate_receiver_thread_ = true;
    
    if (asio_thread_ && asio_thread_->joinable()) {
      asio_thread_->join();
    }
  }

  void SocketWebSocketConnection::on_open(connection_hdl hdl) {
    std::cout << "[WebSocketConnection] Connection opened" << std::endl;
    std::unique_lock<std::mutex> lock(connection_mutex_);
    is_connected_ = true;
    connection_cv_.notify_all();
  }

  void SocketWebSocketConnection::on_close(connection_hdl hdl) {
    std::cout << "[WebSocketConnection] Connection closed" << std::endl;
    is_connected_ = false;
    if (!terminate_receiver_thread_) {
      ReportError(TransportError::R2C_CONNECTION_CLOSED);
    }
  }

  void SocketWebSocketConnection::on_fail(connection_hdl hdl) {
    std::cout << "[WebSocketConnection] Connection failed" << std::endl;
    is_connected_ = false;
    if (!terminate_receiver_thread_) {
      ReportError(TransportError::R2C_SOCKET_ERROR);
    }
    std::unique_lock<std::mutex> lock(connection_mutex_);
    connection_cv_.notify_all();
  }

  void SocketWebSocketConnection::on_message(connection_hdl hdl, message_ptr msg) {
    // Handle JSON messages
    const std::string& payload = msg->get_payload();
    
    json j;
    j.Parse(payload.c_str());
    
    if (j.HasParseError()) {
      std::cout << "[WebSocketConnection] JSON parse error - Ignoring message" << std::endl;
      return;
    }
    
    if (incoming_message_callback_) {
      incoming_message_callback_(j);
    }
  }
}