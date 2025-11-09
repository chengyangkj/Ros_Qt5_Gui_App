#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeSubscribeMsg : public ROSBridgeMsg {
public:
	ROSBridgeSubscribeMsg() : ROSBridgeMsg() {}

	ROSBridgeSubscribeMsg(bool init_opcode) : ROSBridgeMsg()
	{
		if (init_opcode)
			op_ = ROSBridgeMsg::SUBSCRIBE;
	}

	virtual ~ROSBridgeSubscribeMsg() = default;


	// Subscribe messages will never be received from the client
	// So we don't need to fill this instance from JSON or other wire-level representations
	bool FromJSON(const rapidjson::Document &data) = delete;
	bool FromBSON(bson_t &bson) = delete;

	rapidjson::Document ToJSON(rapidjson::Document::AllocatorType& alloc)
	{
		rapidjson::Document d(rapidjson::kObjectType);
		d.AddMember("op", getOpCodeString(), alloc);

		add_if_value_changed(d, alloc, "id", id_);
		add_if_value_changed(d, alloc, "topic", topic_);
		add_if_value_changed(d, alloc, "type", type_);
		add_if_value_changed(d, alloc, "queue_length", queue_length_);
		add_if_value_changed(d, alloc, "throttle_rate", throttle_rate_);
		add_if_value_changed(d, alloc, "compression", compression_);

		return d;
	}

	void ToBSON(bson_t &bson)
	{
		BSON_APPEND_UTF8(&bson, "op", getOpCodeString().c_str());
		add_if_value_changed(bson, "id", id_);

		add_if_value_changed(bson, "topic", topic_);
		add_if_value_changed(bson, "type", type_);
		add_if_value_changed(bson, "queue_length", queue_length_);
		add_if_value_changed(bson, "throttle_rate", throttle_rate_);
		add_if_value_changed(bson, "compression", compression_);
	}

	std::string topic_;
	std::string type_;
	int queue_length_ = -1;
	int throttle_rate_ = -1;
	std::string compression_;
private:
	/* data */
};
