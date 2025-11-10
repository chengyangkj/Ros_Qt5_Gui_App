#include "ros_topic.h"

namespace rosbridge2cpp {

	ROSCallbackHandle<FunVrROSPublishMsg> ROSTopic::Subscribe(FunVrROSPublishMsg callback)
	{
		++subscription_counter_;

		// Only send subscribe when this ROSTopic hasn't sent this command before
		if (subscribe_id_ == "") {
			subscribe_id_.append("subscribe:");
			subscribe_id_.append(topic_name_);
			subscribe_id_.append(":");
			subscribe_id_.append(std::to_string(++ros_.id_counter));

			ROSBridgeSubscribeMsg cmd(true);
			cmd.id_ = subscribe_id_;
			cmd.topic_ = topic_name_;
			cmd.type_ = message_type_;
			cmd.compression_ = compression_;
			cmd.throttle_rate_ = throttle_rate_;
			cmd.queue_length_ = queue_size_;

			if (!ros_.SendMessage(cmd))
			{
				subscribe_id_ = "";
			}
		}

		if (subscribe_id_ != "")
		{
			// Register callback in ROSBridge
			ROSCallbackHandle<FunVrROSPublishMsg> handle(callback);
			ros_.RegisterTopicCallback(topic_name_, handle); // Register callback in ROSBridge
			return handle;
		}

		subscribe_id_ = "";
		return ROSCallbackHandle<FunVrROSPublishMsg>();
	}

	bool ROSTopic::Unsubscribe(const ROSCallbackHandle<FunVrROSPublishMsg>& callback_handle)
	{
		// We've no active subscription
		if (subscribe_id_ == "")
			return false;

		if (!ros_.UnregisterTopicCallback(topic_name_, callback_handle)) { // Unregister callback in ROSBridge
			// failed to unregister callback - maybe the method is different from already registered callbacks
			std::cerr << "[ROSTopic] Passed unknown callback to ROSTopic::unsubscribe. This callback is not registered in the ROSBridge instance. Aborting..." << std::endl;
			return false;
		}

		--subscription_counter_;

		if (subscription_counter_ > 0)
			return true;

		std::cout << "[ROSTopic] No callbacks registered anymore - unsubscribe from topic" << std::endl;
		// Handle unsubscription when no callback is registered anymore
		//rapidjson::Document cmd;
		//cmd.SetObject();

		ROSBridgeUnsubscribeMsg cmd(true);
		cmd.id_ = subscribe_id_;
		cmd.topic_ = topic_name_;

		if (ros_.SendMessage(cmd)) {
			subscribe_id_ = "";
			subscription_counter_ = 0; // shouldn't be necessary ...
			return true;
		}
		return false;
	}

	bool ROSTopic::Advertise()
	{
		if (is_advertised_)
			return true;

		advertise_id_ = "";
		advertise_id_.append("advertise:");
		advertise_id_.append(topic_name_);
		advertise_id_.append(":");
		advertise_id_.append(std::to_string(++ros_.id_counter));

		ROSBridgeAdvertiseMsg cmd(true);
		cmd.id_ = advertise_id_;
		cmd.topic_ = topic_name_;
		cmd.type_ = message_type_;
		cmd.latch_ = latch_;
		cmd.queue_size_ = queue_size_;

		if (ros_.SendMessage(cmd)) {
			is_advertised_ = true;
		}
		return is_advertised_;
	}

	bool ROSTopic::Unadvertise()
	{
		if (!is_advertised_)
			return true;

		ROSBridgeUnadvertiseMsg cmd(true);
		cmd.id_ = advertise_id_;
		cmd.topic_ = topic_name_;

		if (ros_.SendMessage(cmd)) {
			is_advertised_ = false;
		}
		return !is_advertised_;
	}

	// void ROSTopic::Publish(json &message){
	//	if(!is_advertised_)
	//	Advertise();

	//   std::string publish_id;
	//   publish_id.append("publish:");
	//   publish_id.append(topic_name_);
	//   publish_id.append(":");
	//   publish_id.append(std::to_string(++ros_.id_counter));

	//   rapidjson::Document cmd;
	//   cmd.SetObject();
	//   cmd.AddMember("op","publish", cmd.GetAllocator());
	//   cmd.AddMember("id", publish_id, cmd.GetAllocator());
	//   cmd.AddMember("topic", topic_name_, cmd.GetAllocator());
	//   cmd.AddMember("msg", message, cmd.GetAllocator());
	//   cmd.AddMember("latch", latch_, cmd.GetAllocator());

	//   std::cout << "[ROSTopic] Publishing data " << Helper::get_string_from_rapidjson(cmd);

	//   ros_.SendMessage(cmd);
	// }

	bool ROSTopic::Publish(rapidjson::Value &message)
	{
		if (!is_advertised_) {
			if (!Advertise()) {
				return false;
			}
		}

		std::string publish_id = GeneratePublishID();

		ROSBridgePublishMsg cmd(true);
		cmd.id_ = publish_id;
		cmd.topic_ = topic_name_;
		cmd.msg_json_ = message;
		cmd.latch_ = latch_;

		//Queue is not implemented for JSON
		return ros_.SendMessage(cmd);
	}


	std::string ROSTopic::GeneratePublishID()
	{
		std::string publish_id;
		publish_id.append("publish:");
		publish_id.append(topic_name_);
		publish_id.append(":");
		publish_id.append(std::to_string(++ros_.id_counter));
		return publish_id;
	}
}
