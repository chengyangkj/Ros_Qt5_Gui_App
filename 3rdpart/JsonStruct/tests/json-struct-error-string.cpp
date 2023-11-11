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
  /* Invalid stuff */
  "One": "foo / bar",
  "Two": "foo \/ bar"
})json";

struct Struct
{
  std::string One;
  std::string Two;
  JS_OBJ(One, Two);
};

TEST_CASE("test_make_error_string", "[json_struct][error]")
{
  JS::ParseContext context(json_data1);
  Struct substruct;
  auto error = context.parseTo(substruct);

  REQUIRE(error != JS::Error::NoError);

  std::string errorString = context.makeErrorString();
  REQUIRE(errorString.size() != 0);
}

TEST_CASE("test_make_error_string_unnasigned_required_member", "[json_struct][error]")
{
  static const char json_data[] = R"json({
  "One": "foo / bar"
  })json";

  JS::ParseContext context(json_data);
  context.allow_missing_members = false;
  context.allow_unasigned_required_members = false;
  Struct substruct;
  auto error = context.parseTo(substruct);

  REQUIRE(error != JS::Error::NoError);

  std::string errorString = context.makeErrorString();
  REQUIRE(errorString.size() != 0);
}

TEST_CASE("test_make_error_string_missing_member", "[json_struct][error]")
{
  static const char json_data[] = R"json({
  "One": "foo / bar",
  "Two": "foo \/ bar",
  "Three": "foo \/ bar"
  })json";

  JS::ParseContext context(json_data);
  context.allow_missing_members = false;
  context.allow_unasigned_required_members = false;
  Struct substruct;
  auto error = context.parseTo(substruct);
  (void) error;
  context.error = JS::Error::MissingPropertyMember;

  REQUIRE(context.error != JS::Error::NoError);

  std::string errorString = context.makeErrorString();
  REQUIRE(errorString.size() != 0);
}

TEST_CASE("test_make_error_string_missing_members", "[json_struct][error]")
{
  static const char json_data[] = R"json({
  "One": "foo / bar",
  "Two": "foo \/ bar",
  "Three": "foo \/ bar",
  "Four": 4232
  })json";

  JS::ParseContext context(json_data);
  context.allow_missing_members = true;
  context.allow_unasigned_required_members = false;
  Struct substruct;
  auto error = context.parseTo(substruct);
  (void)error;
  context.error = JS::Error::MissingPropertyMember;

  REQUIRE(context.error != JS::Error::NoError);

  std::string errorString = context.makeErrorString();
  REQUIRE(errorString.size() != 0);
}

} // namespace
