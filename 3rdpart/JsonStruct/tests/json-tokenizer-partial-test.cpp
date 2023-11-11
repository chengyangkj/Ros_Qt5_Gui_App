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

namespace json_tokenizer_partial_test
{
const char json_data_partial_1_1[] = "{";
const char json_data_partial_1_2[] = "   \"foo\": \"bar\","
                                     "   \"color\": \"red\"\n"
                                     "}";

TEST_CASE("check_json_partial_1", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_1_1, sizeof(json_data_partial_1_1));
  tokenizer.addData(json_data_partial_1_2, sizeof(json_data_partial_1_2));

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
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

const char json_data_partial_2_1[] = "{  \"fo";
const char json_data_partial_2_2[] = "o\": \"bar\","
                                     "   \"color\": \"red\"\n"
                                     "}";

TEST_CASE("check_json_partial_2", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_2_1, sizeof(json_data_partial_2_1));
  tokenizer.addData(json_data_partial_2_2, sizeof(json_data_partial_2_2));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  std::string foo(token.name.data, token.name.size);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::String, "color", JS::Type::String, "red") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

const char json_data_partial_3_1[] = "{  \"foo\"";
const char json_data_partial_3_2[] = ": \"bar\","
                                     "   \"color\": \"red\"\n"
                                     "}";

TEST_CASE("check_json_partial_3", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_3_1, sizeof(json_data_partial_3_1));
  tokenizer.addData(json_data_partial_3_2, sizeof(json_data_partial_3_2));

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
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

const char json_data_partial_4_1[] = "{  \"foo\": \"bar\"";
const char json_data_partial_4_2[] = ","
                                     "   \"color\": \"red\"\n"
                                     "}";

TEST_CASE("check_json_partial_4", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_4_1, sizeof(json_data_partial_4_1));
  tokenizer.addData(json_data_partial_4_2, sizeof(json_data_partial_4_2));

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
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

const char json_data_partial_5_1[] = "{  \"foo\": \"bar\","
                                     "   col";
const char json_data_partial_5_2[] = "or : \"red\"\n"
                                     "}";

TEST_CASE("check_json_partial_5", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_5_1, sizeof(json_data_partial_5_1));
  tokenizer.addData(json_data_partial_5_2, sizeof(json_data_partial_5_2));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "color", JS::Type::String, "red") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

const char json_data_partial_6_1[] = "{  \"foo\": \"bar\","
                                     "   color : tr";
const char json_data_partial_6_2[] = "ue"
                                     "}";

TEST_CASE("check_json_partial_6", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_6_1, sizeof(json_data_partial_6_1));
  tokenizer.addData(json_data_partial_6_2, sizeof(json_data_partial_6_2));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "color", JS::Type::Bool, "true") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

const char json_data_partial_7_1[] = "{  \"foo\": \"bar\","
                                     "   color : true";
const char json_data_partial_7_2[] = "}";

TEST_CASE("check_json_partial_7", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_7_1, sizeof(json_data_partial_7_1));
  tokenizer.addData(json_data_partial_7_2, sizeof(json_data_partial_7_2));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "color", JS::Type::Bool, "true") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

const char json_data_partial_8_1[] = "{  \"foo\": \"bar\","
                                     "  \"array\": ["
                                     "    \"one\","
                                     "    \"two\",";
const char json_data_partial_8_2[] = "    \"three\""
                                     "  ]"
                                     "}";

TEST_CASE("check_json_partial_8", "[tokenizer]")
{
  JS::Error error;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_8_1, sizeof(json_data_partial_8_1));
  tokenizer.addData(json_data_partial_8_2, sizeof(json_data_partial_8_2));

  JS::Token token;
  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(assert_token(token, JS::Type::String, "foo", JS::Type::String, "bar") == 0);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::String, "array", JS::Type::ArrayStart, "[") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::String, "one") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::String, "two") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::String, "three") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE((assert_token(token, JS::Type::Ascii, "", JS::Type::ArrayEnd, "]") == 0));

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NeedMoreData);
}

TEST_CASE("check_remove_callback", "[tokenizer]")
{
  JS::Error error = JS::Error::NoError;
  JS::Tokenizer tokenizer;
  tokenizer.allowAsciiType(true);
  tokenizer.allowNewLineAsTokenDelimiter(true);
  tokenizer.addData(json_data_partial_8_1, sizeof(json_data_partial_8_1));
  tokenizer.addData(json_data_partial_8_2, sizeof(json_data_partial_8_2));
  JS::Token token;
  bool has_been_called = false;
  {
    auto ref = tokenizer.registerNeedMoreDataCallback([&has_been_called](const JS::Tokenizer &) {
      has_been_called = true;
    });
    error = tokenizer.nextToken(token);
  }

  while (error == JS::Error::NoError && token.value_type != JS::Type::ObjectEnd)
    error = tokenizer.nextToken(token);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(has_been_called == false);
}
} // namespace json_tokenizer_partial_test
