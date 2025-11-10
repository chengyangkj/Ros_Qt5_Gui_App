#pragma once

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

using json = rapidjson::Document;
namespace rosbridge2cpp {
	class Helper {
	public:
		Helper() = default;
		~Helper() = default;

		std::string static get_string_from_rapidjson(json &d)
		{
			rapidjson::StringBuffer buffer;
			rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
			d.Accept(writer);
			return buffer.GetString();
		}

		std::string static get_string_from_rapidjson(const json &d)
		{
			rapidjson::StringBuffer buffer;
			rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
			d.Accept(writer);
			return buffer.GetString();
		}

	};
}
