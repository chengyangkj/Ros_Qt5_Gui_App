#pragma once
#include <functional>

#include "rapidjson/document.h"

#include "messages/rosbridge_msg.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_publish_msg.h"
#include "messages/rosbridge_service_response_msg.h"

namespace rosbridge2cpp {
	using json = rapidjson::Document;
	typedef std::function<void(const json&)> FunVcrJSON;
	// typedef std::function<void(ROSBridgeMsg&)> FunVrROSMSG;
	typedef std::function<void(const ROSBridgePublishMsg&)> FunVrROSPublishMsg;
	typedef std::function<void(ROSBridgeServiceResponseMsg&)> FunVrROSServiceResponseMsg;
	typedef std::function<void(ROSBridgeCallServiceMsg&, rapidjson::Document::AllocatorType&)> FunVrROSCallServiceMsgrROSServiceResponseMsgrAllocator;
	typedef std::function<void(ROSBridgeCallServiceMsg&)> FunVrROSCallServiceMsgrROSServiceResponseMsg;
	// typedef std::function<json(json&)> FunJSONcrJSON;

	enum class TransportError { R2C_SOCKET_ERROR, R2C_CONNECTION_CLOSED };
	extern unsigned long ROSCallbackHandle_id_counter;

	template<typename FunctionType>
	class ROSCallbackHandle {
	public:
		ROSCallbackHandle()
		: id_(0)
		, function_() {}

		ROSCallbackHandle(FunctionType& function)
		: id_(ROSCallbackHandle_id_counter)
		, function_(function)
		{
			if (function_ != nullptr) {
				ROSCallbackHandle_id_counter++;
			} else {
				id_ = 0;
			}
		}

		ROSCallbackHandle(const ROSCallbackHandle& other)
		: id_(other.id_)
		, function_(other.function_) {}

		bool IsValid() const
		{
			return (function_ != nullptr) && (id_ > 0);
		}

		bool operator == (const ROSCallbackHandle& other) const
		{
			return other.id_ == id_;
		}

		bool operator<(const ROSCallbackHandle& other) const
		{
			return other.id_ < id_;
		}

		FunctionType& GetFunction()
		{
			return function_;
		}

	private:
		unsigned long id_;
		FunctionType function_;
	};
}
