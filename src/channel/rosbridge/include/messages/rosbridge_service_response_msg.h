#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeServiceResponseMsg : public ROSBridgeMsg {
public:
	ROSBridgeServiceResponseMsg() : ROSBridgeMsg() {}

	ROSBridgeServiceResponseMsg(bool init_opcode) : ROSBridgeMsg()
	{
		if (init_opcode)
			op_ = ROSBridgeMsg::SERVICE_RESPONSE;
	}

	virtual ~ROSBridgeServiceResponseMsg()
	{
		if (values_bson_ != nullptr)
			bson_destroy(values_bson_);
		if (full_msg_bson_ != nullptr)
			bson_destroy(full_msg_bson_);
	}

	// Warning: This conversion moves the 'values' field
	// out of the given JSON data into this class
	// 'values' will become null afterwards.
	//
	// This method parses the "service", "result" and "values" fields from
	// incoming publish messages into this class
	bool FromJSON(rapidjson::Document &data)
	{
		if (!ROSBridgeMsg::FromJSON(data))
			return false;

		if (!data.HasMember("service")) {
			std::cerr << "[ROSBridgeServiceResponseMsg] Received 'service_response' message without 'service' field." << std::endl; // TODO: use generic logging
			return false;
		}

		service_ = data["service"].GetString();

		if (!data.HasMember("result")) {
			std::cerr << "[ROSBridgeServiceResponseMsg] Received 'service_response' message without 'result' field." << std::endl;
			return false;
		}

		result_ = data["result"].GetBool();

		if (!data.HasMember("values")) {
			return true; // return true, since args is optional. Other parameters will not be set right now
		}

		values_json_ = data["values"];

		return true;
	}

	bool FromBSON(bson_t &bson)
	{
		if (!ROSBridgeMsg::FromBSON(bson))
			return false;

		if (!bson_has_field(&bson, "service")) {
			std::cerr << "[ROSBridgeServiceResponseMsg] Received 'service_response' message without 'service' field." << std::endl;
			return false;
		}

		bool key_found = false;
		service_ = rosbridge2cpp::Helper::get_utf8_by_key("service", bson, key_found);
		assert(key_found);
		key_found = false;

		if (!bson_has_field(&bson, "result")) {
			std::cerr << "[ROSBridgeServiceResponseMsg] Received 'service_response' message without 'result' field." << std::endl;
			return false;
		}

		result_ = rosbridge2cpp::Helper::get_bool_by_key("result", bson, key_found);
		assert(key_found);
		key_found = false;

		if (!bson_has_field(&bson, "values")) {
			return true;
		}

		full_msg_bson_ = &bson;

		return true;
	}

	rapidjson::Document ToJSON(rapidjson::Document::AllocatorType& alloc)
	{
		rapidjson::Document d(rapidjson::kObjectType);
		d.AddMember("op", getOpCodeString(), alloc);
		add_if_value_changed(d, alloc, "id", id_);
		add_if_value_changed(d, alloc, "service", service_);

		d.AddMember("result", result_, alloc);

		if (!values_json_.IsNull())
			d.AddMember("values", values_json_, alloc);
		return d;
	}

	void ToBSON(bson_t &bson)
	{
		BSON_APPEND_UTF8(&bson, "op", getOpCodeString().c_str());
		add_if_value_changed(bson, "id", id_);

		add_if_value_changed(bson, "service", service_);

		BSON_APPEND_BOOL(&bson, "result", result_);
		if (values_bson_ != nullptr && !BSON_APPEND_DOCUMENT(&bson, "values", values_bson_)) {
			std::cerr << "Error while appending 'values' bson to messge BSON" << std::endl;
		}
	}

	std::string service_;
	bool result_ = false;
	// The json data in the different wire-level representations
	rapidjson::Value values_json_;
	bson_t *values_bson_ = nullptr;

	// WARNING:
	// In contrast to the other bson variable above,
	// this BSON instance will contain the full
	// received rosbridge Message
	// when FromBSON has been called in bson_only_mode
	//
	// This is due to the absence of robust pointers to subdocuments
	// in bson that are still valid to use when the parent document
	// might get modified.
	//
	// This ptr will be set, when ToBSON is called and the ServiceResponse carries 'values'
	// Otherwise, it stays as a nullptr
	bson_t *full_msg_bson_ = nullptr;


private:
	/* data */
};
