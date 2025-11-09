
#include "ros_bridge.h"
#include "ros_topic.h"
#include <bson.h>

namespace rosbridge2cpp {

	static const std::chrono::seconds SendThreadFreezeTimeout = std::chrono::seconds(5);
	unsigned long ROSCallbackHandle_id_counter = 1;

	ROSBridge::~ROSBridge()
	{
		run_publisher_queue_thread_ = false;
		if (publisher_queue_thread_.joinable())
		{
			bool waitForThread = (std::chrono::system_clock::now() - LastDataSendTime < SendThreadFreezeTimeout);
			if (waitForThread)
			{
				publisher_queue_thread_.join();
			}
			else
			{
				publisher_queue_thread_.detach();
			}
		}

		for (auto& queue : publisher_queues_)
		{
			while (queue.size())
			{
				bson_destroy(queue.front());
				queue.pop();
			}
		}
	}

	bool ROSBridge::SendMessage(std::string data) {
		spinlock::scoped_lock_wait_for_short_task lock(transport_layer_access_mutex_);
		return transport_layer_.SendMessage(data);
	}

	bool ROSBridge::SendMessage(json &data)
	{
		if (bson_only_mode()) {
			// going from JSON to BSON
			std::string str_repr = Helper::get_string_from_rapidjson(data);
			std::cout << "[ROSBridge] serializing from JSON to BSON for: " << str_repr << std::endl;
			// return transport_layer_.SendMessage(data,length);

			bson_t bson;
			bson_error_t error;
			if (!bson_init_from_json(&bson, str_repr.c_str(), -1, &error)) {
				printf("bson_init_from_json() failed: %s\n", error.message);
				bson_destroy(&bson);
				return false;
			}
			const uint8_t *bson_data = bson_get_data(&bson);
			uint32_t bson_size = bson.len;
			spinlock::scoped_lock_wait_for_short_task lock(transport_layer_access_mutex_);
			bool retval = transport_layer_.SendMessage(bson_data, bson_size);
			bson_destroy(&bson);
			return retval;
		}
		else {
			std::string str_repr = Helper::get_string_from_rapidjson(data);
			return SendMessage(str_repr);
		}
	}

	bool ROSBridge::SendMessage(ROSBridgeMsg &msg)
	{
		if (bson_only_mode()) {
			bson_t message = BSON_INITIALIZER;
			msg.ToBSON(message);
			//size_t offset;

			const uint8_t *bson_data = bson_get_data(&message);
			uint32_t bson_size = message.len;
			spinlock::scoped_lock_wait_for_short_task lock(transport_layer_access_mutex_);
			bool retval = transport_layer_.SendMessage(bson_data, bson_size);
			bson_destroy(&message); // TODO needed?
			return retval;

			// // going from JSON to BSON
			// std::string str_repr = Helper::get_string_from_rapidjson(data);
			// std::cout << "[ROSBridge] serializing from JSON to BSON for: " << str_repr << std::endl;
			// // return transport_layer_.SendMessage(data,length);
			//
			// bson_t bson;
			// bson_error_t error;
			// if (!bson_init_from_json(&bson, str_repr.c_str(), -1, &error)) {
			//   printf("bson_init_from_json() failed: %s\n", error.message);
			//   bson_destroy(&bson);
			//   return false;
			// }
			// const uint8_t *bson_data = bson_get_data (&bson);
			// uint32_t bson_size = bson.len;
			// bool retval = transport_layer_.SendMessage(bson_data,bson_size);
			// bson_destroy(&bson);
			// std::cerr << "Not implemented" << std::endl;
			// return false;
		}


		// Convert ROSBridgeMsg to JSON
		json alloc;
		json message = msg.ToJSON(alloc.GetAllocator());

		std::string str_repr = Helper::get_string_from_rapidjson(message);
		return SendMessage(str_repr);
	}

	bool ROSBridge::QueueMessage(const std::string& topic_name, size_t queue_size, ROSBridgePublishMsg& msg)
	{
		assert(bson_only_mode_); // queueing is not supported for json data

		if (!run_publisher_queue_thread_)
		{
			return false;
		}

		bson_t* message = bson_new();
		bson_init(message);
		msg.ToBSON(*message);

		{
			spinlock::scoped_lock_wait_for_short_task lock(change_publisher_queues_mutex_);
			if (publisher_topics_.find(topic_name) == publisher_topics_.end())
			{
				publisher_topics_[topic_name] = publisher_queues_.size();
				publisher_queues_.push_back(std::queue<bson_t*>());
			}

			auto& queue = publisher_queues_[publisher_topics_[topic_name]];
			if (queue_size > 0 && queue.size() >= queue_size) // make space if necessary
			{
				bson_destroy(queue.front());
				queue.pop();
			}

			queue.push(message);
		}

		return true;
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

		if (bson_only_mode()) {
			if (!data.full_msg_bson_) {
				std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << ", but full message field is missing. Aborting" << std::endl;
				return;
			}
		}
		else {
			if (data.msg_json_.IsNull()) {
				std::cerr << "[ROSBridge] Received message for topic " << incoming_topic_name << ", but 'msg' field is missing. Aborting" << std::endl;
				return;
			}
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

		if (bson_only_mode()) {
			auto service_request_callback_it = registered_service_request_callbacks_bson_.find(incoming_service);

			if (service_request_callback_it == registered_service_request_callbacks_bson_.end()) {
				std::cerr << "[ROSBridge] Received service request for service :" << incoming_service << " where no callback has been registered before" << std::endl;
				return;
			}
			service_request_callback_it->second(data);
		}
		else
		{
			auto service_request_callback_it = registered_service_request_callbacks_.find(incoming_service);

			if (service_request_callback_it == registered_service_request_callbacks_.end()) {
				std::cerr << "[ROSBridge] Received service request for service :" << incoming_service << " where no bson callback has been registered before" << std::endl;
				return;
			}
			rapidjson::Document response_allocator;

			// Execute the callback for the given service id
			service_request_callback_it->second(data, response_allocator.GetAllocator());
		}
	}

	// void ROSBridge::HandleIncomingMessage(ROSBridgeMsg &msg) {}

	void ROSBridge::IncomingMessageCallback(bson_t &bson)
	{
		//ROSBridgeMsg msg;
		//msg.FromBSON(bson);
		//HandleIncomingMessage(msg);

		// Check the message type and dispatch the message properly
		//
		// Incoming Topic messages
		bool key_found = false;

		if (Helper::get_utf8_by_key("op", bson, key_found) == "publish") {
			ROSBridgePublishMsg m;
			if (m.FromBSON(bson)) {
				HandleIncomingPublishMessage(m);
				return;
			}

			std::cerr << "Failed to parse publish message into class. Skipping message." << std::endl;
		}

		// Service responses for service we called earlier
		if (Helper::get_utf8_by_key("op", bson, key_found) == "service_response") {
			ROSBridgeServiceResponseMsg m;
			if (m.FromBSON(bson)) {
				HandleIncomingServiceResponseMessage(m);
				return;
			}
			std::cerr << "Failed to parse service_response message into class. Skipping message." << std::endl;
		}

		// Service Requests to a service that we advertised in ROSService
		if (Helper::get_utf8_by_key("op", bson, key_found) == "call_service") {
			ROSBridgeCallServiceMsg m;
			m.FromBSON(bson);
			HandleIncomingServiceRequestMessage(m);
		}
	}

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
		if (bson_only_mode()) {
			auto fun = [this](bson_t &bson) { IncomingMessageCallback(bson); };

			transport_layer_.SetTransportMode(ITransportLayer::BSON);
			transport_layer_.RegisterIncomingMessageCallback(fun);
		}
		else {
			// JSON mode
			auto fun = [this](json &document) { IncomingMessageCallback(document); };
			transport_layer_.RegisterIncomingMessageCallback(fun);
		}

		run_publisher_queue_thread_ = true;
		publisher_queue_thread_ = std::thread(&ROSBridge::RunPublisherQueueThread, this);

		return transport_layer_.Init(ip_addr, port);
	}

	bool ROSBridge::IsHealthy() const
	{
		return run_publisher_queue_thread_ &&
			(std::chrono::system_clock::now() - LastDataSendTime < SendThreadFreezeTimeout);
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

	void ROSBridge::RegisterServiceRequestCallback(std::string service_name, FunVrROSCallServiceMsgrROSServiceResponseMsg fun)
	{
		registered_service_request_callbacks_bson_[service_name] = fun;
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

	int ROSBridge::RunPublisherQueueThread()
	{
		int return_value = 0;
		int num_retries_left = 10;
		float sleep_duration = 0.2f;

		while (run_publisher_queue_thread_)
		{
			LastDataSendTime = std::chrono::system_clock::now();

			if (sleep_duration > 0.0f)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((long long)(sleep_duration * 1000000.0)));
				sleep_duration = 0.0f;
			}

			bson_t* msg;
			{
				spinlock::scoped_lock_wait_for_short_task lock(change_publisher_queues_mutex_);
				current_publisher_queue_++;
				if (current_publisher_queue_ >= publisher_queues_.size())
				{
					current_publisher_queue_ = 0;
					// Enforce sleep once every topic was handled to allow
					// synchronous ROSBridge calls (e.g. Subscribe, Advertise).
					sleep_duration = 0.01f;

					if (publisher_queues_.size() == 0)
					{
						sleep_duration = 0.1f;
						continue;
					}
				}
				auto& queue = publisher_queues_[current_publisher_queue_];
				if (queue.size())
				{
					msg = queue.front();
					queue.pop();
				}
				else
				{
					continue;
				}
			}

			const uint8_t* bson_data = bson_get_data(msg);
			uint32_t bson_size = msg->len;
			{
				spinlock::scoped_lock_wait_for_long_task lock(transport_layer_access_mutex_);
				const bool success = transport_layer_.SendMessage(bson_data, bson_size);
				bson_destroy(msg);
				if (!success)
				{
					num_retries_left--;
					sleep_duration = 0.2f;
					if (num_retries_left <= 0) {
						run_publisher_queue_thread_ = false;
						return_value = 2;
						std::cout << "[ROSBridge] Lost connection to ROSBridge!" << std::endl;
					}
				}
				else
				{
					num_retries_left = 10;
				}
			}
		}

		return return_value;
	}
}
