#pragma once

#include "rapidjson/document.h"

using json = rapidjson::Document;

/*
 * This class creates json variables
 * that contain the structure of
 * popular ROS Messages.
 */
namespace rosbridge2cpp {
	class ROSMessageFactory {
	public:
		ROSMessageFactory() = default;
		~ROSMessageFactory() = default;

		static json std_msgs_header(json::AllocatorType& allocator)
		{
			json header(rapidjson::kObjectType);
			header.AddMember("seq", (uint32_t)0, allocator);
			rapidjson::Value stamp(rapidjson::kObjectType);
			stamp.AddMember("secs", (uint64_t)0, allocator);
			stamp.AddMember("nsecs", (uint64_t)0, allocator);
			header.AddMember("stamp", stamp, allocator);
			header.AddMember("frame_id", std::string(""), allocator);

			return header;
		}


		static json geometry_msgs_vector3(json::AllocatorType& allocator)
		{
			json msg(rapidjson::kObjectType);
			msg.AddMember("x", (double)0, allocator);
			msg.AddMember("y", (double)0, allocator);
			msg.AddMember("z", (double)0, allocator);

			return msg;
		}

		static json geometry_msgs_quaternion(json::AllocatorType& allocator)
		{
			json msg(rapidjson::kObjectType);
			msg.AddMember("x", (double)0, allocator);
			msg.AddMember("y", (double)0, allocator);
			msg.AddMember("z", (double)0, allocator);
			msg.AddMember("w", (double)1, allocator);

			return msg;
		}

		static json geometry_msgs_transform(json::AllocatorType& allocator)
		{
			json msg(rapidjson::kObjectType);
			msg.AddMember("translation", geometry_msgs_vector3(allocator), allocator);
			msg.AddMember("rotation", geometry_msgs_quaternion(allocator), allocator);

			return msg;
		}

		static json geometry_msgs_transformstamped(json::AllocatorType& allocator)
		{
			json msg(rapidjson::kObjectType);
			msg.AddMember("header", std_msgs_header(allocator), allocator);
			msg.AddMember("child_frame_id", std::string(""), allocator);
			msg.AddMember("transform", geometry_msgs_transform(allocator), allocator);

			return msg;
		}

		static json sensor_msgs_image(json::AllocatorType& allocator)
		{
			json msg(rapidjson::kObjectType);
			msg.AddMember("header", std_msgs_header(allocator), allocator);
			msg.AddMember("height", (uint32_t)0, allocator);
			msg.AddMember("width", (uint32_t)0, allocator);
			msg.AddMember("encoding", std::string(""), allocator);
			msg.AddMember("is_bigendian", (uint32_t)0, allocator);
			msg.AddMember("step", (uint32_t)0, allocator);
			msg.AddMember("data", std::string(""), allocator); // uint8[] will be represented as a base64 string in rosbridge
			//msg.AddMember("child_frame_id", std::string(""), allocator);
			//msg.AddMember("transform", geometry_msgs_transform(allocator), allocator);

			return msg;
		}
	};
}
