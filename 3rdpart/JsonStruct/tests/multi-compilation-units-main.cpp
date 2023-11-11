/*
 * Copyright © 2020 Øystein Myrmo
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

#include "catch2/catch.hpp"
#include <json_struct/json_struct.h>
#include <string>

namespace multiple_compilation_units
{
const char json_data1[] = R"json(
  {
    "num1": 1.234,
    "num2": 5.678
  }
)json";

const char json_data2[] = R"json(
  {
    "num1": 4.321,
    "num2": 8.765
  }
)json";

extern std::string serialize_json1(float num1, double num2);
extern bool deserialize_json1(const std::string &json);
extern std::string serialize_json2(float num1, double num2);
extern bool deserialize_json2(const std::string &json);

TEST_CASE("test_multiple_compilation_units", "[json_struct]")
{
  std::string json1 = serialize_json1(1.111f, 2.222);
  std::string json2 = serialize_json2(1.111f, 2.222);
  REQUIRE(json1 == json2);

  bool ok1 = deserialize_json1(json_data1);
  bool ok2 = deserialize_json2(json_data2);
  REQUIRE(ok1);
  REQUIRE(ok2);
}
} // namespace multiple_compilation_units
