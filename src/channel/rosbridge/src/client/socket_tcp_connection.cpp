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

  bool SocketTCPConnection::SendMessage(const uint8_t *data, unsigned int length){
    if( send(sock_ , data , length , 0) < 0)
    {
      perror("[TCPConnection] Send failed : ");
      return false;
    }
    std::cout<<"[TCPConnection] Data send ("<<length<<" Bytes): " << std::endl;
    for (int i = 0; i < length; i++) {
      std::cout << ":" << std::setw(2) << std::setfill('0') << std::hex << (int)( data[i] );
    }
    std::cout<<"[TCPConnection] Data end" << std::endl;
    std::cout << std::endl;
    return true;
  }

  int SocketTCPConnection::ReceiverThreadFunction(){

    std::cout<<"[TCPConnection] Receiving\n";
    std::cout<<"[TCPConnection] bson_only_mode is : " << bson_only_mode_ << std::endl;

    uint32_t buf_size=1000000; // 1 MB
    std::unique_ptr<unsigned char[]> recv_buffer(new unsigned char[buf_size]); 

    // Register message callback
    // std::function<void(const json)> message_cb = messageCallback;

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
      if(bson_only_mode_){
        for (int i = 0; i < count; i++) {
          std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << (int)( recv_buffer[i] );
        }
        std::cout << "[TCPConnection] Received Data end" << std::endl;
        std::cout << std::endl;
      }else{
        // Print the human-readable data
        printf("%.*s", count, recv_buffer.get());
      }

      // TODO catch parse error properly
      json j;
      bson_t b;

      if(bson_only_mode_){
        // std::cerr << "bson receive not implemented right now" << std::endl;
        if(!bson_init_static (&b, recv_buffer.get(), count)){
          std::cout << "[TCPConnection] Error on BSON parse - Ignoring message" << std::endl;
          continue;
        }
        if(incoming_message_callback_bson_){
          incoming_message_callback_bson_(b);
        }

        continue;
        /*
        if(!bson_init_static (&b, recv_buffer.get(), count)){
          std::cout << "Error on BSON parse - Ignoring message" << std::endl;
          continue;
        }
        if(incoming_message_callback_bson_){
          incoming_message_callback_bson_(b);
        }
        std::string str = bson_as_json (&b, NULL);
        // if (!bson_init_from_json(&bson, str_repr.c_str(), -1, &error)) {
        //   printf("bson_init_from_json() failed: %s\n", error.message);
        //   bson_destroy(&bson);
        //   return false;
        // }
        // const uint8_t *bson_data = bson_get_data (&bson);
        // uint32_t bson_size = bson.len;
        // bool retval = transport_layer_.SendMessage(bson_data,bson_size);
        // bson_destroy(&b);
        j.Parse(str);
        */
      }else{
        j.Parse((char *)recv_buffer.get());

        // TODO Use a thread for the message callback?
        if(incoming_message_callback_){
          incoming_message_callback_(j);
        }
      }



      if(bson_only_mode_)
        bson_destroy(&b);
      std::cout.flush();
    }
    return 0; // Everything went OK - terminateReceiverThread is now true
  }

  void SocketTCPConnection::RegisterIncomingMessageCallback(std::function<void(json&)> fun){
    // TODO unify with report_error
    incoming_message_callback_ = fun;
    callback_function_defined_ = true;
  }

  void SocketTCPConnection::RegisterIncomingMessageCallback(std::function<void(bson_t&)> fun){
    // TODO unify with report_error
    incoming_message_callback_bson_ = fun;
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
    switch(mode){
      case ITransportLayer::JSON:
        bson_only_mode_ = false;
      break;
      case ITransportLayer::BSON:
        bson_only_mode_ = true;
      break;
      default:
        std::cerr << "Given TransportMode Not implemented " << std::endl;
    }
  }
}
