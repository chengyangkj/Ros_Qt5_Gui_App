/*
* Copyright © 2022 Jonathan Poncelet

* Permission to use, copy, modify, distribute, and sell this software and its
* documentation for any purpose is hereby granted without fee, provided that
* the above copyright notice appear in all copies and that both that copyright
* notice and this permission notice appear in supporting documentation, and
* that the name of the copyright holders not be used in advertising or
* publicity pertaining to distribution of the software without specific,
* written prior permission.  The copyright holders make no representations
* about the suitability of this software for any purpose.  It is provided "as
* is" without express or implied warranty.

* THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
* INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
* EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
* CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
* DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
* TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
* OF THIS SOFTWARE.
*/
#include <string>
#include <cstdint>
#include <json_struct/json_struct.h>
#include "catch2/catch.hpp"

static constexpr const char* const JSON_ONE_NESTED_OBJECT = R"json(
{
  "object1":
  {
    "value": 1,
    "nested_object":
    {
      "nested_1":
      {
        "some_value": "foo"
      }
    }
  },
  "object2":
  {
    "value": 2
  }
})json";

static constexpr const char* const JSON_TWO_NESTED_OBJECTS = R"json(
{
  "object1":
  {
    "value": 1,
    "nested_object":
    {
      "nested_1":
      {
        "some_value": "foo"
      },
      "nested_2":
      {
        "some_value": "bar"
      }
    }
  },
  "object2":
  {
    "value": 2
  }
})json";

struct NestedObject
{
  struct Inner
  {
    std::string some_value;

    JS_OBJ(some_value);
  };

  Inner nested_1;
  Inner nested_2;

  JS_OBJ(nested_1, nested_2);
};

struct Object1
{
  int32_t value = 0;
  NestedObject nested_object;

  JS_OBJ(value, nested_object);
};

struct Object1_OmittedVal
{
  int32_t value = 0;
  // NestedObject is deliberately omitted here.

  JS_OBJ(value);
};

struct Object2
{
  int32_t value = 0;

  JS_OBJ(value);
};

struct Container
{
  Object1 object1;
  Object2 object2;

  JS_OBJ(object1, object2);
};

struct Container_OmittedVal
{
  Object1_OmittedVal object1;
  Object2 object2;

  JS_OBJ(object1, object2);
};

template<typename T>
static bool TryParse(const std::string& json)
{
  static const int32_t EXPECTED_CONTAINER_VALUE = 2;

  JS::ParseContext context(json);
  T container;
  JS::Error parseError = context.parseTo(container);

  bool parsedSuccessfully = parseError == JS::Error::NoError;
  bool containerValueCorrect = container.object2.value == EXPECTED_CONTAINER_VALUE;
  return parsedSuccessfully && containerValueCorrect;
}

TEST_CASE("nested_optional", "json_struct")
{
  REQUIRE(TryParse<Container>(JSON_ONE_NESTED_OBJECT));
  REQUIRE(TryParse<Container>(JSON_TWO_NESTED_OBJECTS));
  REQUIRE(TryParse<Container_OmittedVal>(JSON_ONE_NESTED_OBJECT));
  REQUIRE(TryParse<Container_OmittedVal>(JSON_TWO_NESTED_OBJECTS));
}

