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

#include <json_struct/json_struct.h>

#include "catch2/catch.hpp"
#include <sstream>
#include <string>

namespace
{

struct NullTerminatorStruct
{
  std::string data;

  JS_OBJECT(JS_MEMBER(data));
};

std::string create_string_with_nullterminators()
{
  std::stringstream content;
  content.write("Hello", 5);
  int num = 5;
  content.write((char *)&num, 4);
  content.write(" World!", 7);
  return content.str();
}

std::string create_json_with_nullterminators()
{
  std::stringstream content;
  content.write("{\"data\":\"Hello", 14);
  int num = 5;
  content.write((char *)&num, 4);
  content.write(" World!\"}", 9);
  return content.str();
}

void check_parse_nullterminated_string(const std::string jsonWithNullTerminator)
{
  NullTerminatorStruct nullStruct;

  JS::ParseContext pc(jsonWithNullTerminator);
  auto error = pc.parseTo(nullStruct);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(nullStruct.data.size() == 16);
}

TEST_CASE("check_parse_nullterminated_string", "[json_struct]")
{
  std::string jsonWithNullTerminator = create_json_with_nullterminators();
  check_parse_nullterminated_string(jsonWithNullTerminator);
}

TEST_CASE("check_serialize_nullterminated_struct", "[json_struct]")
{
  NullTerminatorStruct nullStruct;
  nullStruct.data = create_string_with_nullterminators();
  std::string json = JS::serializeStruct(nullStruct);
  check_parse_nullterminated_string(json);
}

} // namespace
