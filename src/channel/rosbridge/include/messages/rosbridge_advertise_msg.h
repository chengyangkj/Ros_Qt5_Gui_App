#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeAdvertiseMsg : public ROSBridgeMsg {
public:
	ROSBridgeAdvertiseMsg () : ROSBridgeMsg() {}

	ROSBridgeAdvertiseMsg(bool init_opcode) : ROSBridgeMsg()
	{
		if (init_opcode)
			op_ = ROSBridgeMsg::ADVERTISE;
	}

	virtual ~ROSBridgeAdvertiseMsg() = default;

	// Advertise messages will never be received from the client
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
		add_if_value_changed(d, alloc, "queue_size", queue_size_);

		d.AddMember("latch", latch_, alloc);

		return d;
	}

	void ToBSON(bson_t &bson)
	{
		BSON_APPEND_UTF8(&bson, "op", getOpCodeString().c_str());

		add_if_value_changed(bson, "id", id_);
		add_if_value_changed(bson, "topic", topic_);
		add_if_value_changed(bson, "type", type_);

		add_if_value_changed(bson, "queue_size", queue_size_);

		//d.AddMember("latch", latch_, alloc);
		BSON_APPEND_BOOL(&bson, "latch", latch_);
	}

	std::string topic_;
	std::string type_;
	//std::string compression_ = "none";
	//int throttle_rate_ = 0;
	bool latch_ = false;
	int queue_size_ = -1;
private:
	/* data */
};
