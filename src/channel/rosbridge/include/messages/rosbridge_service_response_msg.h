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

	virtual ~ROSBridgeServiceResponseMsg() = default;

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

	std::string service_;
	bool result_ = false;
	// The json data in the different wire-level representations
	rapidjson::Value values_json_;


private:
	/* data */
};
