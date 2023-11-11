/*
 * Copyright © 2021 Jorgen Lind
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
"num1": 32587.403333333333333333,
"num2": 32587.403333333333333333
}
)json";

struct Json1
{
  float num1;
  double num2;

  JS_OBJ(num1, num2);
};

TEST_CASE("json_struct_float", "[json_struct][float]")
{
  JS::ParseContext context(json_data);
  Json1 json;
  auto error = context.parseTo(json);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(json.num1 == 32587.403333333333333333f);
  REQUIRE(json.num2 == 32587.403333333333333333);
  // context.parseTo(missing);
  // std::string out = JS::serializeStruct(missing);
}
} // namespace
