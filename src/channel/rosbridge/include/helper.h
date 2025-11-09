#pragma once

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <bson.h>

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

		// dot_notation refers to MongoDB dot notation
		// returns "" and sets success to true if suitable data can't be found via the dot notation
		std::string static get_utf8_by_key(const char *dot_notation, bson_t &b, bool &success)
		{
			bson_iter_t iter;
			bson_iter_t val;

			if (bson_iter_init(&iter, &b) &&
				bson_iter_find_descendant(&iter, dot_notation, &val) &&
				BSON_ITER_HOLDS_UTF8(&val)) {
				success = true;
				return std::string(bson_iter_utf8(&val, NULL));
			}

			success = false;
			return "";
		}

		// dot_notation refers to MongoDB dot notation
		// returns INT32_MAX and sets success to 'false' if suitable data can't be found via the dot notation
		int32_t static get_int32_by_key(const char *dot_notation, bson_t &b, bool &success)
		{
			bson_iter_t iter;
			bson_iter_t val;

			if (bson_iter_init(&iter, &b) &&
				bson_iter_find_descendant(&iter, dot_notation, &val) &&
				BSON_ITER_HOLDS_INT32(&val)) {
				success = true;
				return bson_iter_int32(&val);
			}
			success = false;
			return INT32_MAX;
		}

		// dot_notation refers to MongoDB dot notation
		// returns -1 and sets success to 'false' if suitable data can't be found via the dot notation
		double static get_double_by_key(const char *dot_notation, bson_t &b, bool &success)
		{
			bson_iter_t iter;
			bson_iter_t val;

			// if (bson_iter_init (&iter, &b) &&
			//	 bson_iter_find_descendant (&iter, dot_notation, &val) &&
			//	 BSON_ITER_HOLDS_DOUBLE (&val)) {
			if (bson_iter_init(&iter, &b) &&
				bson_iter_find_descendant(&iter, dot_notation, &val)) {
				if (!BSON_ITER_HOLDS_DOUBLE(&val)) {
					std::cout << "Key found, but not double typed" << std::endl;
				}
				else {
					//std::cout << "Key " << dot_notation << " found. success is = " << success << std::endl;
					//std::cout << "Key " << dot_notation << " found." << std::endl;
					success = true;
					//std::cout << "								 success is now = " << success << std::endl;
					return bson_iter_double(&val);
				}
			}
			std::cout << "Couldn't find descendant" << std::endl;
			success = false;
			return -1.0;
		}

		// dot_notation refers to MongoDB dot notation
		// returns false and sets success to 'false' if suitable data can't be found via the dot notation
		bool static get_bool_by_key(const char *dot_notation, bson_t &b, bool &success)
		{
			bson_iter_t iter;
			bson_iter_t val;

			if (bson_iter_init(&iter, &b) &&
				bson_iter_find_descendant(&iter, dot_notation, &val) &&
				BSON_ITER_HOLDS_BOOL(&val)) {
				success = true;
				return bson_iter_bool(&val);
			}
			success = false;
			return false;
		}

		// dot_notation refers to MongoDB dot notation
		// returns nullptr and sets success to 'false' if suitable data can't be found via the dot notation
		//
		// binary_data_length holds the size of the buffer where the returned pointer points to.
		static const uint8_t * get_binary_by_key(const char *dot_notation, bson_t &b, uint32_t &binary_data_length, bool &success)
		{
			bson_iter_t iter;
			bson_iter_t val;

			if (bson_iter_init(&iter, &b) &&
				bson_iter_find_descendant(&iter, dot_notation, &val) &&
				BSON_ITER_HOLDS_BINARY(&val)) {
				bson_subtype_t subtype;
				const uint8_t *binary;

				bson_iter_binary(&val, &subtype, &binary_data_length, &binary);
				assert(binary);
				success = true;
				return binary;
			}
			success = false;
			return nullptr;
		}

		// dot_notation refers to MongoDB dot notation
		// returns nullptr and sets success to 'false' if suitable data can't be found via the dot notation
		//
		// array_size holds the size in byte of the buffer where the returned pointer points to.
		static const uint8_t * get_array_by_key(const char *dot_notation, bson_t &b, uint32_t &array_size, bool &success)
		{
			bson_iter_t iter;
			bson_iter_t val;

			if (bson_iter_init(&iter, &b) &&
				bson_iter_find_descendant(&iter, dot_notation, &val) &&
				BSON_ITER_HOLDS_ARRAY(&val)) {
				const uint8_t *binary;

				bson_iter_array(&val, &array_size, &binary);
				assert(binary);
				success = true;
				return binary;
			}
			success = false;
			return nullptr;
		}

		bool static bson_has_key(bson_t &b, const char *key)
		{
			return bson_has_field(&b, key);
		}
	};
}
