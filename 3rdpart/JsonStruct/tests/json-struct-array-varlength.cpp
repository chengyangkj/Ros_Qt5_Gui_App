/*
 * Copyright � 2021 Jorgen Lind
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

#include <json_struct/json_struct.h>

#include "catch2/catch.hpp"

namespace
{
const char json_data[] = R"json(
{
  "vec": [
    1,2,3,4
  ]
}
)json";

struct JsonObject1
{
  JS::ArrayVariableContent<int, 10> vec;
  JS_OBJ(vec);
};

struct JsonObject2
{
  JS::ArrayVariableContent<int, 2> vec;
  JS_OBJ(vec);
};

TEST_CASE("array_variable_content", "[json_struct][array]")
{
  JS::ParseContext context(json_data);
  JsonObject1 object;
  REQUIRE(context.parseTo(object) == JS::Error::NoError);

  REQUIRE(object.vec.size == 4);
  REQUIRE(object.vec.data[0] == 1);
  REQUIRE(object.vec.data[1] == 2);
  REQUIRE(object.vec.data[2] == 3);
  REQUIRE(object.vec.data[3] == 4);
}

TEST_CASE("array_variable_content_fail", "[json_struct][array]")
{
  JS::ParseContext context(json_data);
  JsonObject2 object;
  REQUIRE(context.parseTo(object) != JS::Error::NoError);
}
}
