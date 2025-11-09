#pragma once

#include <iostream>

#include "messages/rosbridge_msg.h"

class ROSBridgeAdvertiseServiceMsg : public ROSBridgeMsg{
public:
	ROSBridgeAdvertiseServiceMsg() : ROSBridgeMsg() {}

	ROSBridgeAdvertiseServiceMsg(bool init_opcode) : ROSBridgeMsg() {
		if (init_opcode)
			op_ = ROSBridgeMsg::ADVERTISE_SERVICE;
	}

	virtual ~ROSBridgeAdvertiseServiceMsg() = default;

	// Advertise messages will never be received from the client
	// So we don't need to fill this instance from JSON or other wire-level representations
	bool FromJSON(const rapidjson::Document &data) = delete;
	bool FromBSON(bson_t &bson) = delete;

	rapidjson::Document ToJSON(rapidjson::Document::AllocatorType& alloc)
	{
		rapidjson::Document d(rapidjson::kObjectType);
		d.AddMember("op", getOpCodeString(), alloc);

		add_if_value_changed(d, alloc, "id", id_);
		add_if_value_changed(d, alloc, "service", service_);
		add_if_value_changed(d, alloc, "type", type_);
		return d;
	}

	void ToBSON(bson_t &bson)
	{
		BSON_APPEND_UTF8(&bson, "op", getOpCodeString().c_str());

		add_if_value_changed(bson, "id", id_);
		add_if_value_changed(bson, "service", service_);
		add_if_value_changed(bson, "type", type_);
	}


	std::string service_;
	std::string type_;
private:
	/* data */
};
