#pragma once

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <unordered_map>

#include <bson.h>

#include "helper.h"


/*
 * The base class for all ROSBridge messages
 *
 * Incoming Messages will be parsed to this class
 */

class ROSBridgeMsg {
public:
	enum OpCode {
		OPCODE_UNDEFINED, // Default value, before parsing

		FRAGMENT, // not implemented currently
		PNG, // not implemented currently
		SET_LEVEL, // not implemented currently
		STATUS, // not implemented currently
		AUTH, // not implemented currently
		ADVERTISE,
		UNADVERTISE,
		PUBLISH,
		SUBSCRIBE,
		UNSUBSCRIBE,
		ADVERTISE_SERVICE,
		UNADVERTISE_SERVICE,
		CALL_SERVICE,
		SERVICE_RESPONSE
	};

	std::unordered_map<std::string, OpCode> op_code_mapping = {
		{"fragment", FRAGMENT},
		{"png", PNG},
		{"set_level", SET_LEVEL},
		{"status", STATUS},
		{"auth", AUTH},
		{"advertise", ADVERTISE},
		{"unadvertise", UNADVERTISE},
		{"publish", PUBLISH},
		{"subscribe", SUBSCRIBE},
		{"unsubscribe", UNSUBSCRIBE},
		{"advertise_service", ADVERTISE_SERVICE},
		{"unadvertise_service", UNADVERTISE_SERVICE},
		{"call_service", CALL_SERVICE},
		{"service_response", SERVICE_RESPONSE}
	};

	// std::unordered_map<OpCode, std::string> reverse_op_code_mapping = {
	//   {OPCODE_UNDEFINED,"opcode_undefined"},
	//   {FRAGMENT,"fragment"},
	//   {PNG,"png"},
	//   {SET_LEVEL,"set_level"},
	//   {STATUS, "status"},
	//   {AUTH, "auth"},
	//   {ADVERTISE, "advertise"},
	//   {UNADVERTISE, "unadvertise"},
	//   {PUBLISH, "publish"},
	//   {SUBSCRIBE, "subscribe"},
	//   {UNSUBSCRIBE, "unsubscribe"},
	//   {ADVERTISE_SERVICE, "advertise_service"},
	//   {UNADVERTISE_SERVICE, "unadvertise_service"},
	//   {CALL_SERVICE, "call_service"},
	//   {SERVICE_RESPONSE, "service_response"}
	// };

	ROSBridgeMsg() = default;

	// This method can be used to parse incoming ROSBridge messages in order
	// to fill the class variables from the wire representation (for example, JSON or BSON)
	//
	// Returns true if a 'op' field could be found in the given data package
	//
	// This method will care about the 'op' and 'id' key
	// 'op' is mandatory, while 'id' is optional
	bool FromJSON(const rapidjson::Document &data)
	{
		if (!data.HasMember("op")) {
			std::cerr << "[ROSBridgeMsg] Received message without 'op' field" << std::endl;
			return false;
		}

		std::string op_code = data["op"].GetString();
		auto mapping_iterator = op_code_mapping.find(op_code);
		if (mapping_iterator == op_code_mapping.end()) {
			std::cerr << "[ROSBridgeMsg] Received message with invalid 'op' field: " << op_code << std::endl;
			return false;
		}

		op_ = mapping_iterator->second;

		if (!data.HasMember("id"))
			return true; // return true, because id is only optional

		id_ = data["id"].GetString();

		return true;
	}

	bool FromBSON(bson_t &bson)
	{
		if (!bson_has_field(&bson, "op")) {
			std::cerr << "[ROSBridgeMsg] Received message without 'op' field" << std::endl;
			return false;
		}

		bool key_found = false;
		std::string op_code = rosbridge2cpp::Helper::get_utf8_by_key("op", bson, key_found);
		assert(key_found); // should always be true, otherwise this is contradictory to the !bson_has_field check
		key_found = false;

		auto mapping_iterator = op_code_mapping.find(op_code);
		if (mapping_iterator == op_code_mapping.end()) {
			std::cerr << "[ROSBridgeMsg] Received message with invalid 'op' field: " << op_code << std::endl;
			return false;
		}

		op_ = mapping_iterator->second;

		if (!bson_has_field(&bson, "id"))
			return true; // return true, because id is only optional

		id_ = rosbridge2cpp::Helper::get_utf8_by_key("id", bson, key_found);
		assert(key_found);

		return true;
	}

	std::string getOpCodeString()
	{
		if (op_ == OPCODE_UNDEFINED) return "opcode_undefined";
		if (op_ == FRAGMENT) return "fragment";
		if (op_ == PNG) return "png";
		if (op_ == SET_LEVEL) return "set_level";
		if (op_ == STATUS) return  "status";
		if (op_ == AUTH) return  "auth";
		if (op_ == ADVERTISE) return  "advertise";
		if (op_ == UNADVERTISE) return  "unadvertise";
		if (op_ == PUBLISH) return  "publish";
		if (op_ == SUBSCRIBE) return  "subscribe";
		if (op_ == UNSUBSCRIBE) return  "unsubscribe";
		if (op_ == ADVERTISE_SERVICE) return  "advertise_service";
		if (op_ == UNADVERTISE_SERVICE) return  "unadvertise_service";
		if (op_ == CALL_SERVICE) return  "call_service";
		if (op_ == SERVICE_RESPONSE) return  "service_response";
		return "";
	}

	virtual ~ROSBridgeMsg() = default;

	virtual rapidjson::Document ToJSON(rapidjson::Document::AllocatorType& alloc) = 0;
	virtual void ToBSON(bson_t& bson) = 0;

	OpCode op_ = OPCODE_UNDEFINED;
	std::string id_ = "";

protected:
	// key must be valid as long as 'd' lives!
	void add_if_value_changed(rapidjson::Document &d, rapidjson::Document::AllocatorType& alloc, const char* key, const std::string& value)
	{
		if (!value.empty())
			d.AddMember(rapidjson::StringRef(key), value, alloc);
	}

	// key must be valid as long as 'd' lives!
	void add_if_value_changed(rapidjson::Document &d, rapidjson::Document::AllocatorType& alloc, const char* key, int value)
	{
		if (value != -1)
			d.AddMember(rapidjson::StringRef(key), value, alloc);
	}

	void add_if_value_changed(bson_t &bson, const char* key, const std::string& value)
	{
		if (!value.empty())
			BSON_APPEND_UTF8(&bson, key, value.c_str());
	}

	// key must be valid as long as 'd' lives!
	void add_if_value_changed(bson_t &bson, const char* key, int value)
	{
		if (value != -1)
			BSON_APPEND_INT32(&bson, key, value);
	}

private:
	/* data */
};
