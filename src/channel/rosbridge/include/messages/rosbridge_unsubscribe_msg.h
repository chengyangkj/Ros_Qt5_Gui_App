#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeUnsubscribeMsg : public ROSBridgeMsg {
public:
	ROSBridgeUnsubscribeMsg() : ROSBridgeMsg() {}

	ROSBridgeUnsubscribeMsg(bool init_opcode) : ROSBridgeMsg()
	{
		if (init_opcode)
			op_ = ROSBridgeMsg::UNSUBSCRIBE;
	}

	virtual ~ROSBridgeUnsubscribeMsg() = default;


	// Unsubscribe messages will never be received from the client
	// So we don't need to fill this instance from JSON or other wire-level representations
	bool FromJSON(const rapidjson::Document &data) = delete;
	bool FromBSON(bson_t &bson) = delete;

	rapidjson::Document ToJSON(rapidjson::Document::AllocatorType& alloc)
	{
		rapidjson::Document d(rapidjson::kObjectType);
		d.AddMember("op", getOpCodeString(), alloc);
		add_if_value_changed(d, alloc, "id", id_);
		add_if_value_changed(d, alloc, "topic", topic_);
		return d;
	}

	void ToBSON(bson_t &bson)
	{
		BSON_APPEND_UTF8(&bson, "op", getOpCodeString().c_str());
		add_if_value_changed(bson, "id", id_);

		add_if_value_changed(bson, "topic", topic_);
	}

	std::string topic_;
private:
	/* data */
};
