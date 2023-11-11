/*
 * Copyright � 2022 Jorgen Lind
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <stdint.h>
#include <json_struct/json_struct.h>

#include "catch2/catch.hpp"


namespace
{
static const char json[] = R"json({
  "int8": 4,
  "uint8": 5,
  "int16": "6",
  "uint16": "7",
  "int32": "8",
  "uint32": "9",
  "int64": "10",
  "uint64": "11"
})json";

struct stdinttypes
{
  int8_t int8;
  uint8_t uint8;
  int16_t int16;
  uint16_t uint16;
  int32_t int32;
  uint32_t uint32;
  int64_t int64;
  uint64_t uint64;
  JS_OBJ(int8, uint8, int16, uint16, int32, uint32, int64, uint64);
};
TEST_CASE("js_std_int", "json_struct")
{
  JS::ParseContext context(json, sizeof(json));
  context.allow_missing_members = false;
  context.allow_unasigned_required_members = false;

  stdinttypes to_struct;
  auto error = context.parseTo(to_struct);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(to_struct.int8 == 4);
  REQUIRE(to_struct.uint32 == 9);
}

TEST_CASE("large_number_roundtrip", "json_struct")
{
  stdinttypes to_serialize;
  to_serialize.uint64 = 0;
  to_serialize.uint64 = ~to_serialize.uint64;
  std::string serialized = JS::serializeStruct(to_serialize);
  JS::ParseContext context(serialized);

  stdinttypes to_struct;
  auto error = context.parseTo(to_struct);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(to_serialize.uint64 == to_struct.uint64);
}
}

#if defined(__SIZEOF_INT128__)
#define JS_INT_128
#include <json_struct/json_struct.h>
namespace
{
struct very_large_int
{
  JS::js_int128_t data;
  JS_OBJ(data);
};

TEST_CASE("test_128_int", "json_struct")
{
  very_large_int large_int;
  large_int.data = 1;
  large_int.data <<= 127;
  large_int.data = ~large_int.data;

  std::string large_int_json = JS::serializeStruct(large_int);

  very_large_int large_int_target;
  JS::ParseContext pc(large_int_json);
  auto error = pc.parseTo(large_int_target);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(large_int_target.data == large_int.data);
}
}
#endif
