/*
 * Copyright © 2017 Jorgen Lind
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

#include <cmrc/cmrc.hpp>

CMRC_DECLARE(external_json);

namespace
{

struct Simple
{
  std::string A;
  bool b;
  int some_longer_name;
  JS_OBJECT(JS_MEMBER(A), JS_MEMBER(b), JS_MEMBER(some_longer_name));
};

const char expected1[] = R"json({
  "A": "TestString",
  "b": false,
  "some_longer_name": 456
})json";

TEST_CASE("test_serialize_simple", "[json_struct][serialize]")
{
  Simple simple;
  simple.A = "TestString";
  simple.b = false;
  simple.some_longer_name = 456;

  std::string output = JS::serializeStruct(simple);
  REQUIRE(output == expected1);
}

struct A
{
  int a;
  JS_OBJECT(JS_MEMBER(a));
};

struct B : public A
{
  float b;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(A)), JS_MEMBER(b));
};

struct D
{
  int d;
  JS_OBJECT(JS_MEMBER(d));
};

struct E : public D
{
  double e;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(D)), JS_MEMBER(e));
};

struct F : public E
{
  int f;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(E)), JS_MEMBER(f));
};
struct G
{
  std::string g;
  JS_OBJECT(JS_MEMBER(g));
};

struct Subclass : public B, public F, public G
{
  int h;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(B), JS_SUPER_CLASS(F), JS_SUPER_CLASS(G)), JS_MEMBER(h));
};

const char expected2[] = R"json({
  "h": 7,
  "g": "OutputString",
  "f": 6,
  "e": 5.5,
  "d": 5,
  "b": 4.5,
  "a": 4
})json";

TEST_CASE("test_serialize_deep", "[json_struct][serialize]")
{
  Subclass subclass;
  subclass.a = 4;
  subclass.b = 4.5;
  subclass.d = 5;
  subclass.e = 5.5;
  subclass.f = 6;
  subclass.g = "OutputString";
  subclass.h = 7;

  std::string output = JS::serializeStruct(subclass);
  REQUIRE(output == expected2);
}

struct WithEscapedData
{
  std::string data;
  JS_OBJECT(JS_MEMBER(data));
};

const char escaped_expected[] = R"json({
  "data": "escaped \n \" \t string"
})json";

TEST_CASE("serialze_test_escaped_data", "[json_struct][serialize]")
{
  WithEscapedData escaped;
  escaped.data = "escaped \n \" \t string";
  std::string output = JS::serializeStruct(escaped);
  REQUIRE(output == escaped_expected);
}

const char expected3[] = R"json({"h":7,"g":"OutputString","f":6,"e":5.5,"d":5,"b":4.5,"a":4})json";
TEST_CASE("serialize_test_compact", "[json_struct][serialize]")
{
  Subclass subclass;
  subclass.a = 4;
  subclass.b = 4.5;
  subclass.d = 5;
  subclass.e = 5.5;
  subclass.f = 6;
  subclass.g = "OutputString";
  subclass.h = 7;

  std::string output = JS::serializeStruct(subclass, JS::SerializerOptions(JS::SerializerOptions::Compact));
  REQUIRE(output == expected3);
}

TEST_CASE("test_serialize_big", "[json_struct][serialize]")
{
  auto fs = cmrc::external_json::get_filesystem();
  auto generated = fs.open("generated.json");

  JS::JsonObjectOrArrayRef objOrArr;
  {
    JS::ParseContext pc(generated.begin(), generated.size());
    auto error = pc.parseTo(objOrArr);
    REQUIRE(error == JS::Error::NoError);
  }

  std::string serialized_json = JS::serializeStruct(objOrArr);

  {
    JS::ParseContext pc(serialized_json.data(), serialized_json.size());
    auto error = pc.parseTo(objOrArr);
    REQUIRE(error == JS::Error::NoError);
  }
}

const char empty_string_json[] = R"json({
  "key1": "value1",
  "key2": "",
  "key3": "value3"
})json";

struct empty_string_struct
{
  std::string key1;
  std::string key2;
  std::string key3;

  JS_OBJ(key1, key2, key3);
};
TEST_CASE("test_serialize_empty_string", "[json_struct][serialize]")
{
  JS::JsonTokens tokens;
  JS::ParseContext context(empty_string_json);
  auto error = context.parseTo(tokens);
  REQUIRE(error == JS::Error::NoError);

  empty_string_struct empty_struct;
  JS::ParseContext context2(empty_string_json);
  error = context2.parseTo(empty_struct);
  REQUIRE(error == JS::Error::NoError);

  std::string out = JS::serializeStruct(tokens);

  std::string out2 = JS::serializeStruct(empty_struct);
  REQUIRE(out == out2);
  REQUIRE(out == empty_string_json);
}

} // namespace
