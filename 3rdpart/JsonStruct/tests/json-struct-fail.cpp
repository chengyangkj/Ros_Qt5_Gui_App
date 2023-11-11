/*
 * Copyright © 2019 Jorgen Lind
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
  "some_data": 44.50
}
)json";

struct JsonMissingMeta
{
  float missing_meta;
};

struct JsonMissingTypeHandler
{
  double vec[3];
};

struct JsonContainingStructMissingTypeHandler
{
  JsonMissingTypeHandler data;

  JS_OBJECT(JS_MEMBER(data));
};

TEST_CASE("json_struct_error_check_missing_meta", "[json_struct][error]")
{
  JS::ParseContext context(json_data);
  JsonMissingMeta missing;
  JS_UNUSED(context);
  JS_UNUSED(missing);
  // context.parseTo(missing);
  // std::string out = JS::serializeStruct(missing);
}

TEST_CASE("json_struct_error_check_missing_typehandler", "[json_struct][error]")
{
  JS::ParseContext context(json_data);
  JsonContainingStructMissingTypeHandler missing;
  JS_UNUSED(context);
  JS_UNUSED(missing);
  // context.parseTo(missing);
  // std::string out = JS::serializeStruct(missing);
}
} // namespace
