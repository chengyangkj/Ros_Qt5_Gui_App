/*
 * Copyright � 2020 Jorgen Lind
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

#include <optional>
#define JS_STD_OPTIONAL
#include <json_struct/json_struct.h>

#include "catch2/catch.hpp"
#include <unordered_map>


namespace
{
static const char json[] = R"json({
  "Field1": 4,
  "Field2": true,
  "Field3": "432"
})json";

struct field_struct
{
  std::optional<int> Field1;
  std::optional<bool> Field2;
  std::optional<std::string> Field3;
  std::optional<std::string> Field4;
  JS_OBJ(Field1, Field2, Field3, Field4);
};
TEST_CASE("js_std_optional", "json_struct")
{
  JS::ParseContext context(json, sizeof(json));
  context.allow_missing_members = false;
  context.allow_unnasigned_required_members = false;

  field_struct to_struct;
  context.parseTo(to_struct);

  REQUIRE(Field1 == 4);
  REQUIRE(Field2 == true);
  REQUIRE(Field3 == "432");
  REQUIRE(context.missing_members.size() == 0);
  REQUIRE(context.unassigned_required_members.size() == 0);
}
}
#endif
