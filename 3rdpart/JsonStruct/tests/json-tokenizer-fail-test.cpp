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

#include <json_struct/json_struct.h>
#include "tokenizer-test-util.h"

#include "catch2/catch.hpp"

namespace json_tokenizer_fail_test
{
const char json_with_ascii_property[] = "{"
                                        "   \"foo\": \"bar\","
                                        "   color : \"red\""
                                        "}";

TEST_CASE("check_fail_json_with_ascii_property", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.addData(json_with_ascii_property);

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::IllegalPropertyName);
}

const char json_with_ascii_data[] = "{"
                                    "   \"foo\": \"bar\","
                                    "   \"color\": red"
                                    "}";

TEST_CASE("check_fail_json_with_ascii_data", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.addData(json_with_ascii_data);

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::IllegalDataValue);
}

const char json_with_new_line_seperator[] = "{"
                                            "   \"foo\": \"bar\"\n"
                                            "   \"color\": \"red\""
                                            "}";

TEST_CASE("check_fail_json_with_new_line_seperator", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.addData(json_with_new_line_seperator);

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::InvalidToken);
}

const char json_with_comma_before_obj_end[] = "{"
                                              "   \"foo\": \"bar\","
                                              "   \"color\": \"red\","
                                              "}";

TEST_CASE("check_fail_json_with_comma_before_obj_end", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.addData(json_with_comma_before_obj_end, sizeof(json_with_comma_before_obj_end));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "color", JS::Type::String, "red") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::ExpectedDataToken);
}

const char json_with_illegal_chars[] = "{"
                                       "   \"foo\": \"bar\","
                                       " ,  \"color\": \"red\","
                                       "}";

TEST_CASE("check_fail_json_with_illegal_chars", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.addData(json_with_illegal_chars, sizeof(json_with_illegal_chars));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::EncounteredIllegalChar);
}

const char json_with_illegal_comma_in_array[] = "{"
                                                "  \"foo\": [,4,5,6]"
                                                "}";
TEST_CASE("check_fail_json_with_empty_array", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.addData(json_with_illegal_comma_in_array);

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ArrayStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::EncounteredIllegalChar);
}

const char json_broken_stream[] =
  R"json("r::load","line":640,"level":0,"type":"Io","io":{"id":928,"action":0,"state":0,"uri":"http://datamons")json";

TEST_CASE("check_fail_broken_json_stream", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.addData(json_broken_stream);

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error != JS::Error::NoError);
}

} // namespace json_tokenizer_fail_test
