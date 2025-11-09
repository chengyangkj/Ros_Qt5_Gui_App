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

	virtual ~ROSBridgePublishMsg()
	{
		if (msg_bson_ != nullptr)
			bson_destroy(msg_bson_);
		if (full_msg_bson_ != nullptr)
			bson_destroy(full_msg_bson_);
	}

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

	bool FromBSON(bson_t &bson)
	{
		if (!ROSBridgeMsg::FromBSON(bson))
			return false;

		if (!bson_has_field(&bson, "topic")) {
			std::cerr << "[ROSBridgePublishMsg] Received 'publish' message without 'topic' field." << std::endl;
			return false;
		}

		bool key_found = false;
		topic_ = rosbridge2cpp::Helper::get_utf8_by_key("topic", bson, key_found);
		assert(key_found);
		key_found = false;

		if (!bson_has_field(&bson, "msg")) {
			std::cerr << "[ROSBridgePublishMsg] Received 'publish' message without 'msg' field." << std::endl;
			return false;
		}

		full_msg_bson_ = &bson;

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

	void ToBSON(bson_t &bson)
	{
		BSON_APPEND_UTF8(&bson, "op", getOpCodeString().c_str());
		add_if_value_changed(bson, "id", id_);

		add_if_value_changed(bson, "topic", topic_);
		add_if_value_changed(bson, "type", type_);

		BSON_APPEND_BOOL(&bson, "latch", latch_);
		if (msg_bson_ != nullptr && !BSON_APPEND_DOCUMENT(&bson, "msg", msg_bson_)) {
			std::cerr << "Error while appending 'msg' bson to messge BSON" << std::endl;
		}
	}

	std::string topic_;
	std::string type_;
	// std::string compression_;
	// std::string throttle_rate_;
	// std::string queue_length_;
	bool latch_ = false;

	// The json data in the different wire-level representations
	rapidjson::Value msg_json_;

	bson_t *msg_bson_ = nullptr;

	// WARNING:
	// In contrast to the other bson variable above,
	// this BSON instance will contain the full
	// received rosbridge Message
	// when FromBSON has been called in bson_only_mode
	//
	// This is due to the absence of robust pointers to subdocuments
	// in bson that are still valid to use when the parent document
	// might get modified.
	bson_t *full_msg_bson_ = nullptr;

private:
	/* data */
};
