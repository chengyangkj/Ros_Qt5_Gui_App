/*
 * Copyright © 2012 Jorgen Lind
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

#include "json-test-data.h"
#include <json_struct/json_struct.h>
#include "tokenizer-test-util.h"

#include "catch2/catch.hpp"

namespace json_tokenizer_test
{

TEST_CASE("check_json_with_string_and_ascii", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data1, sizeof(json_data1));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::String, "color", JS::Type::String, "red") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "weather", JS::Type::String, "clear") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "weather1", JS::Type::String, "clear1") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ToBeTrue", JS::Type::Bool, "true") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "HeresANull", JS::Type::Null, "null") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsFalse", JS::Type::Bool, "false") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "EscapedString", JS::Type::String, "contains \\\"") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::String, "\\\"EscapedName\\\"", JS::Type::Bool, "true") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::String, "EscapedProp", JS::Type::String, "\\\"Hello\\\"") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsANumber", JS::Type::Number, "3.14") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsAnObject", JS::Type::ObjectStart, "{") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsASubType", JS::Type::String, "red") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "AnotherProp", JS::Type::String, "prop") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsAnotherObject", JS::Type::ObjectStart, "{") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsAnotherASubType", JS::Type::String, "blue") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsAnArray", JS::Type::ArrayStart, "[") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::Number, "12.4") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::Number, "3") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::Number, "43.2") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ArrayEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "ThisIsAnObjectArray", JS::Type::ArrayStart, "[") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::ObjectStart, "{") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "Test1", JS::Type::String, "Test2") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "Test3", JS::Type::String, "Test4") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::ObjectStart, "{") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "Test5", JS::Type::Bool, "true") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "Test7", JS::Type::Bool, "false") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ArrayEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

} // namespace json_tokenizer_test
