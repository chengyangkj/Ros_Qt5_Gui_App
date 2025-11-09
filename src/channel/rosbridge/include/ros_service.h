#pragma once

#include <list>

#include "rapidjson/document.h"

#include "ros_bridge.h"
#include "types.h"
#include "messages/rosbridge_advertise_service_msg.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_unadvertise_service_msg.h"

using json = rapidjson::Document;

namespace rosbridge2cpp {
	class ROSService {
	public:
		// TODO: Implement setter of other options
		ROSService(ROSBridge &ros, std::string service_name, std::string service_type) :
			ros_(ros), service_name_(service_name), service_type_(service_type) {
		}

		// Advertise a service and define the request handling callback
		// The given callback will handle all service requests.
		//
		// The callback will get the CallService packet from the server and 
		// a reference to a ServiceResponse packet, that can be filled as desired.
		//
		// This method will only be used when using rapidjson, since we need to use
		// the allocator of rapidjson to avoid copy operations.
		bool Advertise(FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator callback);
		bool Advertise(FunVrROSCallServiceMsgrROSServiceResponseMsg callback);

		// Unadvertise an advertised service
		// Will do nothing if no service has been advertised before in this instance
		bool Unadvertise();

		// TODO failedCallback parameter
		// Call a ROS-Service
		// The given callback variable will be called when the service reply
		// has been received by ROSBridge. It will passed the received data to the callback.
		// The whole content of the "request" parameter will be send as the "args"
		// argument of the Service Request
		bool CallService(rapidjson::Value &request, FunVrROSServiceResponseMsg callback);
		bool CallService(bson_t *request, FunVrROSServiceResponseMsg callback);

		std::string GenerateServiceCallID();

		std::string ServiceName() {
			return service_name_;
		}

	private:
		ROSBridge &ros_;
		std::string service_name_;
		std::string service_type_;
		bool is_advertised_ = false;
	};
}
