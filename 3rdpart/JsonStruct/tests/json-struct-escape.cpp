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

static const char json_data1[] = R"json({
  "One": "foo / bar",
  "Two": "foo \/ bar"
})json";

struct Struct
{
  std::string One;
  std::string Two;
  JS_OBJ(One, Two);
};

TEST_CASE("test_forward_slash", "[json_struct][escape]")
{
  JS::ParseContext context(json_data1);
  Struct substruct;
  auto error = context.parseTo(substruct);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(substruct.One == "foo / bar");
  REQUIRE(substruct.Two == "foo / bar");

  std::string out = JS::serializeStruct(substruct);

  Struct second_struct;
  JS::ParseContext context2(out);
  error = context2.parseTo(second_struct);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(substruct.One == second_struct.One);
  REQUIRE(substruct.Two == second_struct.Two);


}
} // namespace
