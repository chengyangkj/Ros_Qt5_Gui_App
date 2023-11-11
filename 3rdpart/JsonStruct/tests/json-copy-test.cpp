/*
 * Copyright © 2016 Jorgen Lind
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
const char json[] = "{\n"
                    "  \"property_one\": 432432,\n"
                    "  \"execute_one\": {\n"
                    "    \"number\": 45,\n"
                    "    \"valid\": \"false\"\n"
                    "  },"
                    "  \"execute_two\": 99,\n"
                    "  \"execute_three\": [\n"
                    "    4,\n"
                    "    6,\n"
                    "    8\n"
                    "  ]\n"
                    "}\n";

struct SubObject
{
  int number;
  bool valid;

  JS_OBJECT(JS_MEMBER(number), JS_MEMBER(valid));
};

void js_validate_json(JS::Tokenizer &tokenizer)
{
  JS::Token token;
  JS::Error error;
  std::string buffer;

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::Number);

  error = tokenizer.nextToken(token);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectStart);
  tokenizer.copyFromValue(token, buffer);

  while (error == JS::Error::NoError && token.value_type != JS::Type::ObjectEnd)
    error = tokenizer.nextToken(token);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(token.value_type == JS::Type::ObjectEnd);
  tokenizer.copyIncludingValue(token, buffer);

  while (error == JS::Error::NoError && token.value_type != JS::Type::ObjectEnd)
    error = tokenizer.nextToken(token);

  JS::ParseContext context(buffer.c_str(), buffer.size());
  SubObject subObj;
  error = context.parseTo(subObj);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(subObj.number == 45);
  REQUIRE(subObj.valid == false);
}
TEST_CASE("copy_test_js_copy_full", "[tokenizer]")
{
  JS::Tokenizer tokenizer;
  tokenizer.addData(json);
  js_validate_json(tokenizer);
}

TEST_CASE("copy_test_js_partial_1", "[tokenizer]")
{
  JS::Tokenizer tokenizer;
  tokenizer.addData(json, 40);
  tokenizer.addData(json + 40, sizeof(json) - 40);
  js_validate_json(tokenizer);
}

TEST_CASE("copy_test_js_partial_2", "[tokenizer]")
{
  JS::Tokenizer tokenizer;
  size_t offset = 0;
  std::function<void(JS::Tokenizer &)> func = [&offset](JS::Tokenizer &tok) {
    if (offset + 2 > sizeof(json))
    {
      tok.addData(json + offset, sizeof(json) - offset);
      offset += sizeof(json) - offset;
    }
    else
    {
      tok.addData(json + offset, 2);
      offset += 2;
    }
  };
  auto ref = tokenizer.registerNeedMoreDataCallback(func);

  js_validate_json(tokenizer);
}

TEST_CASE("copy_test_js_partial_3", "[tokenizer]")
{
  JS::Tokenizer tokenizer;
  size_t offset = 0;
  std::function<void(JS::Tokenizer &)> func = [&offset](JS::Tokenizer &tokenizer) {
    if (offset + 1 > sizeof(json))
    {
      tokenizer.addData(json + offset, sizeof(json) - offset);
      offset += sizeof(json) - offset;
    }
    else
    {
      tokenizer.addData(json + offset, 1);
      offset += 1;
    }
  };
  auto ref = tokenizer.registerNeedMoreDataCallback(func);

  js_validate_json(tokenizer);
}

const char json2[] =
  R"json({
  "test": true,
  "more": {
    "sub_object_prop1": true,
    "sub_object_prop2": 456
  },
  "int_value": 65
})json";

struct Child
{
  bool sub_object_prop1;
  int sub_object_prop2;
  JS_OBJECT(JS_MEMBER(sub_object_prop1), JS_MEMBER(sub_object_prop2));
};

struct Parent
{
  bool test;
  Child more;
  int int_value;
  JS_OBJECT(JS_MEMBER(test), JS_MEMBER(more), JS_MEMBER(int_value));
};

TEST_CASE("copy_test_js_copy_parsed", "[tokenizer]")
{
  JS::Tokenizer tokenizer;
  tokenizer.addData(json2);

  JS::Token token;
  JS::Error error = JS::Error::NoError;
  std::vector<JS::Token> tokens;
  while (error == JS::Error::NoError)
  {
    error = tokenizer.nextToken(token);
    tokens.push_back(token);
  }

  JS::ParseContext context;
  context.tokenizer.addData(&tokens);
  Parent parent;
  error = context.parseTo(parent);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(parent.test == true);
  REQUIRE(parent.more.sub_object_prop1 == true);
  REQUIRE(parent.more.sub_object_prop2 == 456);
  REQUIRE(parent.int_value == 65);
}

const char json_token_copy[] = R"json(
{
  "number": 45,
  "valid": false,
  "child": {
    "some_more": "world",
    "another_int": 495
  },
  "more_data": "string data",
  "super_data": "hello"
}
)json";

struct SecondChild
{
  std::string some_more;
  int another_int;
  JS_OBJECT(JS_MEMBER(some_more), JS_MEMBER(another_int));
};
struct SecondParent
{
  int number;
  bool valid;
  JS::JsonTokens child;
  std::string more_data;
  std::string super_data;

  JS_OBJECT(JS_MEMBER(number), JS_MEMBER(valid), JS_MEMBER(child), JS_MEMBER(more_data), JS_MEMBER(super_data));
};

TEST_CASE("copy_test_js_copy_tokens", "[tokenizer]")
{
  SecondParent parent;
  JS::ParseContext parseContext(json_token_copy);
  auto error = parseContext.parseTo(parent);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(parent.child.data.size() == 4);

  JS::ParseContext childContext;
  childContext.tokenizer.addData(&parent.child.data);
  SecondChild child;
  error = childContext.parseTo(child);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(child.another_int == 495);
  REQUIRE(child.some_more == "world");
}
} // namespace
