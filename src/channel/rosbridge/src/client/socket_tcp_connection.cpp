#include "client/socket_tcp_connection.h"

namespace rosbridge2cpp{
  bool SocketTCPConnection::Init(std::string p_ip_addr, int p_port){
    ip_addr_ = p_ip_addr;
    port_ = p_port;

    if (sock_ == -1)
    {
      perror("[TCPConnection] Could not create socket");
    }

    std::cout << "[TCPConnection] Socket created\n";
    // Set up IP address
    connect_to_.sin_addr.s_addr = inet_addr( ip_addr_.c_str() );

    connect_to_.sin_family = AF_INET;
    connect_to_.sin_port = htons( port_ );

    if (connect(sock_ , (struct sockaddr *)&connect_to_ , sizeof(connect_to_)) < 0)
    {
      perror("[TCPConnection] connect failed. Error");
      return false;
    }

    std::cout << "[TCPConnection] Connected\n";

    // Setting up the receiver thread
    std::cout << "[TCPConnection] Setting up receiver thread..." << std::endl;
    receiver_thread_ = std::move(std::thread([=]() {ReceiverThreadFunction(); return 1; }));
    receiver_thread_set_up_ = true;

    return true;
  }
  bool SocketTCPConnection::SendMessage(std::string data){
    if( send(sock_ , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
      perror("[TCPConnection] Send failed : ");
      return false;
    }
    std::cout<<"[TCPConnection] Data send: " << data << "\n";
    return true;
  }


  int SocketTCPConnection::ReceiverThreadFunction(){

    std::cout<<"[TCPConnection] Receiving\n";

    uint32_t buf_size=1000000; // 1 MB
    std::unique_ptr<unsigned char[]> recv_buffer(new unsigned char[buf_size]); 

    // TODO handle joined messages while reading the buffer
    while(!terminate_receiver_thread_){
      std::cout << "." << std::endl;
      int count = recv(sock_, recv_buffer.get(), buf_size, 0);
      std::cout << "[TCPConnection] Received bytes: " << count << std::endl;
      if(count == 0){
        if(!terminate_receiver_thread_)
          ReportError(TransportError ::R2C_CONNECTION_CLOSED);
        return 1; // connection closed
      }
      if(count < 0){
        if(!terminate_receiver_thread_)
          ReportError(TransportError ::R2C_SOCKET_ERROR);
        return 2; // error while receiving from socket
      }

      recv_buffer.get()[count] = 0; // null-terminate to handle it like a c-string
      // Print the human-readable data
      printf("%.*s", count, recv_buffer.get());

      // TODO catch parse error properly
      json j;
      j.Parse((char *)recv_buffer.get());

      // TODO Use a thread for the message callback?
      if(incoming_message_callback_){
        incoming_message_callback_(j);
      }

      std::cout.flush();
    }
    return 0; // Everything went OK - terminateReceiverThread is now true
  }

  void SocketTCPConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun){
    // TODO unify with report_error
    incoming_message_callback_ = fun;
    callback_function_defined_ = true;
  }

  void SocketTCPConnection::RegisterErrorCallback(std::function<void(TransportError)> fun){
    error_callback_ = fun;
  }
  void SocketTCPConnection::ReportError(TransportError err){
    if(error_callback_ == nullptr)
      return;

    error_callback_(err);
  }

  void SocketTCPConnection::SetTransportMode(ITransportLayer::TransportMode mode){
    // Only JSON mode is supported
  }
}
