#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgePublishMsg : public ROSBridgeMsg {
public:
	ROSBridgePublishMsg() : ROSBridgeMsg() {}

	ROSBridgePublishMsg(bool init_opcode) : ROSBridgeMsg()
	{
		if (init_opcode)
			op_ = ROSBridgeMsg::PUBLISH;
	}

	virtual ~ROSBridgePublishMsg() = default;

	// Warning: This conversion moves the 'msg' field
	// out of the given JSON data into this class
	// 'msg' will become null afterwards.
	//
	// This method parses the "topic" and "msg" fields from
	// incoming publish messages into this class
	bool FromJSON(rapidjson::Document &data) {
		if (!ROSBridgeMsg::FromJSON(data))
			return false;

		if (!data.HasMember("topic")) {
			std::cerr << "[ROSBridgePublishMsg] Received 'publish' message without 'topic' field." << std::endl; // TODO: use generic logging
			return false;
		}

		topic_ = data["topic"].GetString();

		if (!data.HasMember("msg")) {
			std::cerr << "[ROSBridgePublishMsg] Received 'publish' message without 'msg' field." << std::endl;
			return false;
		}

		msg_json_ = data["msg"];

		return true;
	}

	rapidjson::Document ToJSON(rapidjson::Document::AllocatorType& alloc)
	{
		rapidjson::Document d(rapidjson::kObjectType);
		d.AddMember("op", getOpCodeString(), alloc);

		add_if_value_changed(d, alloc, "id", id_);
		add_if_value_changed(d, alloc, "topic", topic_);
		add_if_value_changed(d, alloc, "type", type_);


		d.AddMember("latch", latch_, alloc);

		if (!msg_json_.IsNull())
			d.AddMember("msg", msg_json_, alloc);

		return d;
	}

	std::string topic_;
	std::string type_;
	// std::string compression_;
	// std::string throttle_rate_;
	// std::string queue_length_;
	bool latch_ = false;

	// The json data in the different wire-level representations
	rapidjson::Value msg_json_;

private:
	/* data */
};
