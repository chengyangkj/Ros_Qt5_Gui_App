
#include "ros_bridge.h"
#include "ros_topic.h"

namespace rosbridge2cpp {

	static const std::chrono::seconds SendThreadFreezeTimeout = std::chrono::seconds(5);
	unsigned long ROSCallbackHandle_id_counter = 1;

	ROSBridge::~ROSBridge()
	{
	}

	bool ROSBridge::SendMessage(std::string data) {
		spinlock::scoped_lock_wait_for_short_task lock(transport_layer_access_mutex_);
		return transport_layer_.SendMessage(data);
	}

	bool ROSBridge::SendMessage(json &data)
	{
		std::string str_repr = Helper::get_string_from_rapidjson(data);
		return SendMessage(str_repr);
	}

	bool ROSBridge::SendMessage(ROSBridgeMsg &msg)
	{
		// Convert ROSBridgeMsg to JSON
		json alloc;
		json message = msg.ToJSON(alloc.GetAllocator());

		std::string str_repr = Helper::get_string_from_rapidjson(message);
		return SendMessage(str_repr);
	}

	void ROSBridge::HandleIncomingPublishMessage(ROSBridgePublishMsg &data)
	{
		spinlock::scoped_lock_wait_for_short_task lock(change_topics_mutex_);

		// Incoming topic message - dispatch to correct callback
		std::string &incoming_topic_name = data.topic_;
		if (registered_topic_callbacks_.find(incoming_topic_name) == registered_topic_callbacks_.end()) {
			std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << " where no callback has been registered before" << std::endl;
			return;
		}

		if (data.msg_json_.IsNull()) {
			std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << ", but 'msg' field is missing. Aborting" << std::endl;
			return;
		}

		// Iterate over all registered callbacks for the given topic
		for (auto& topic_callback : registered_topic_callbacks_.find(incoming_topic_name)->second) {
			topic_callback.GetFunction()(data);
		}
		return;
	}

	void ROSBridge::HandleIncomingServiceResponseMessage(ROSBridgeServiceResponseMsg &data)
	{
		std::string &incoming_service_id = data.id_;

		auto service_response_callback_it = registered_service_callbacks_.find(incoming_service_id);

		if (service_response_callback_it == registered_service_callbacks_.end()) {
			std::cerr << "[ROSBridge] Received response for service id " << incoming_service_id << "where no callback has been registered before" << std::endl;
			return;
		}

		// Execute the callback for the given service id
		service_response_callback_it->second(data);

		// Delete the callback.
		// Every call_service will create a new id
		registered_service_callbacks_.erase(service_response_callback_it);

	}

	void ROSBridge::HandleIncomingServiceRequestMessage(ROSBridgeCallServiceMsg &data)
	{
		std::string &incoming_service = data.service_;

		auto service_request_callback_it = registered_service_request_callbacks_.find(incoming_service);

		if (service_request_callback_it == registered_service_request_callbacks_.end()) {
			std::cerr << "[ROSBridge] Received service request for service :" << incoming_service << " where no callback has been registered before" << std::endl;
			return;
		}
		rapidjson::Document response_allocator;

		// Execute the callback for the given service id
		service_request_callback_it->second(data, response_allocator.GetAllocator());
	}

	// void ROSBridge::HandleIncomingMessage(ROSBridgeMsg &msg) {}

	void ROSBridge::IncomingMessageCallback(json &data)
	{
		std::string str_repr = Helper::get_string_from_rapidjson(data);

		// Check the message type and dispatch the message properly
		//
		// Incoming Topic messages
		if (std::string(data["op"].GetString(), data["op"].GetStringLength()) == "publish") {
			ROSBridgePublishMsg m;
			if (m.FromJSON(data)) {
				HandleIncomingPublishMessage(m);
				return;
			}

			std::cerr << "Failed to parse publish message into class. Skipping message." << std::endl;
		}

		// Service responses for service we called earlier
		if (std::string(data["op"].GetString(), data["op"].GetStringLength()) == "service_response") {
			ROSBridgeServiceResponseMsg m;
			// m.FromJSON(data);
			if (m.FromJSON(data)) {
				HandleIncomingServiceResponseMessage(m);
				return;
			}
			std::cerr << "Failed to parse service_response message into class. Skipping message." << std::endl;
		}

		// Service Requests to a service that we advertised in ROSService
		if (std::string(data["op"].GetString(), data["op"].GetStringLength()) == "call_service") {
			ROSBridgeCallServiceMsg m;
			m.FromJSON(data);
			HandleIncomingServiceRequestMessage(m);
		}
	}

	bool ROSBridge::Init(std::string ip_addr, int port)
	{
		auto fun = [this](json &document) { IncomingMessageCallback(document); };
		transport_layer_.RegisterIncomingMessageCallback(fun);

		return transport_layer_.Init(ip_addr, port);
	}

	bool ROSBridge::IsHealthy() const
	{
		return true;
	}

	void ROSBridge::RegisterTopicCallback(std::string topic_name, ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle)
	{
		spinlock::scoped_lock_wait_for_short_task lock(change_topics_mutex_);
		registered_topic_callbacks_[topic_name].push_back(callback_handle);
	}

	void ROSBridge::RegisterServiceCallback(std::string service_call_id, FunVrROSServiceResponseMsg fun)
	{
		registered_service_callbacks_[service_call_id] = fun;
	}

	void ROSBridge::RegisterServiceRequestCallback(std::string service_name, FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator fun)
	{
		registered_service_request_callbacks_[service_name] = fun;
	}


	bool ROSBridge::UnregisterTopicCallback(std::string topic_name, const ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle)
	{
		spinlock::scoped_lock_wait_for_short_task lock(change_topics_mutex_);

		if (registered_topic_callbacks_.find(topic_name) == registered_topic_callbacks_.end()) {
			std::cerr << "[ROSBridge] UnregisterTopicCallback called but given topic name '" << topic_name << "' not in map." << std::endl;
			return false;
		}

		std::list<ROSCallbackHandle<FunVrROSPublishMsg>> &r_list_of_callbacks = registered_topic_callbacks_.find(topic_name)->second;

		for (std::list<ROSCallbackHandle<FunVrROSPublishMsg>>::iterator topic_callback_it = r_list_of_callbacks.begin();
			topic_callback_it != r_list_of_callbacks.end();
			++topic_callback_it) {

			if (*topic_callback_it == callback_handle) {
				std::cout << "[ROSBridge] Found CB in UnregisterTopicCallback. Deleting it ... " << std::endl;
				r_list_of_callbacks.erase(topic_callback_it);
				return true;
			}
		}
		return false;
	}
}
