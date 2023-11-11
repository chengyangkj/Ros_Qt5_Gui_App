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
static const char json[] = R"json(
{
  "Field1": 4,
  "Field2": true,
  "ComplexFields": { "Hello": 4, "World": 2 },
  "Field3": "432",
  "ComplexFields2": { "SimpleMember": true, "ArrayOfValues": [4, 3, 5, 7], "SubObject": { "SimpleMember": false, "MoreValues": [ "Hello", "World"] } },
  "Field4": 567
}
)json";

struct ComplexFields_t
{
  int Hello;
  int World;
  JS_OBJ(Hello, World);
};

struct SubObject_t
{
  bool SimpleMember;
  std::vector<std::string> MoreValues;
  JS_OBJ(SimpleMember, MoreValues);
};
struct ComplexFields2_t
{
  ComplexFields2_t(bool init)
    : SimpleMember(init)
  {
  }
  bool SimpleMember;
  std::vector<int> ArrayOfValues;
  SubObject_t SubObject;
  JS_OBJ(SimpleMember, ArrayOfValues, SubObject);
};

struct Root_t
{
  Root_t()
    : ComplexFields2(true)
  {
  }
  int Field1;
  bool Field2;
  std::string Field3;
  int Field4;
  ComplexFields_t ComplexFields;
  ComplexFields2_t ComplexFields2;
  JS_OBJ(Field1, Field2, Field3, Field4, ComplexFields, ComplexFields2);
};

struct NewComplexField
{
  std::string Foo;
  std::string Bar;
  JS_OBJ(Foo, Bar);
};

void verify_map_meta(const JS::Map &map)
{
  for (auto &meta : map.meta)
  {
    bool is_complex = (map.tokens.data[meta.position].value_type == JS::Type::ObjectStart ||
            map.tokens.data[meta.position].value_type == JS::Type::ArrayStart);
    REQUIRE(is_complex);
  }

  auto new_meta = JS::metaForTokens(map.tokens);
  REQUIRE(map.meta.size() == new_meta.size());

  for (int i = 0; i < int(map.meta.size()); i++)
  {
    REQUIRE(new_meta[i].children == map.meta[i].children); 
    REQUIRE(new_meta[i].complex_children == map.meta[i].complex_children); 
    REQUIRE(new_meta[i].has_data == map.meta[i].has_data); 
    REQUIRE(new_meta[i].is_array == map.meta[i].is_array); 
    REQUIRE(new_meta[i].position == map.meta[i].position); 
    REQUIRE(new_meta[i].size == map.meta[i].size); 
    REQUIRE(new_meta[i].skip == map.meta[i].skip); 
  }

  for (auto &json_data : map.json_data)
  {
    auto &token = map.tokens.data[json_data.first];
    REQUIRE(token.name.data >= json_data.second.data());
    REQUIRE(token.value.data + token.value.size < json_data.second.data() + json_data.second.size());
  }
}

TEST_CASE("polymorphic_map_basic", "json_struct")
{
  JS::Map map;
  JS::ParseContext pc(json, sizeof(json), map);
  REQUIRE(pc.error == JS::Error::NoError);

  JS::Error error;
  REQUIRE(map.castTo<int>("Field1", pc) == 4);
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(map.castTo<bool>("Field2", pc) == true);
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(map.castTo<std::string>("Field3", pc) == "432");
  REQUIRE(pc.error == JS::Error::NoError);
  ComplexFields_t complexFields1 = map.castTo<ComplexFields_t>("ComplexFields", pc);
  REQUIRE(pc.error == JS::Error::NoError);
  ComplexFields_t complexFields1_2;
  error = map.castToType("ComplexFields", pc, complexFields1_2);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(complexFields1.Hello == complexFields1_2.Hello);
  REQUIRE(complexFields1.World == complexFields1_2.World);
  REQUIRE(complexFields1.Hello == 4);
  REQUIRE(complexFields1.World == 2);

  ComplexFields2_t complexFields2(false);
  error = map.castToType("ComplexFields2", pc, complexFields2);
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(complexFields2.ArrayOfValues[1] == 3);
  REQUIRE(complexFields2.SubObject.MoreValues[1] == "World");

  REQUIRE(map.castTo<int>("Field4", pc) == 567);
  REQUIRE(pc.error == JS::Error::NoError);

  Root_t root = map.castTo<Root_t>(pc);
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(root.Field1 == 4);
  REQUIRE(root.ComplexFields2.SubObject.MoreValues[0] == "Hello");

  auto it = map.find("Field2");
  REQUIRE(it != map.end());
  REQUIRE(it->value_type == JS::Type::Bool);

  map.setValue("HelloWorld", pc, uint64_t(4567));
  REQUIRE(map.castTo<int>("HelloWorld", pc) == 4567);
  REQUIRE(pc.error == JS::Error::NoError);
  verify_map_meta(map);

  map.setValue("Field4", pc, std::string("hello world"));
  REQUIRE(map.castTo<std::string>("Field4", pc) == "hello world");
  REQUIRE(pc.error == JS::Error::NoError);
  verify_map_meta(map);

  map.setValue("ComplexFields", pc, true);
  REQUIRE(map.castTo<bool>("ComplexFields", pc) == true);
  REQUIRE(pc.error == JS::Error::NoError);
  verify_map_meta(map);

  NewComplexField newComplexField;
  newComplexField.Foo = "Map is awsome";
  newComplexField.Bar = "NewComplexField";
  map.setValue("ComplexFields", pc, newComplexField);

  NewComplexField newComplexField2;
  REQUIRE(map.castToType("ComplexFields", pc, newComplexField2) == JS::Error::NoError);
  REQUIRE(newComplexField.Foo == newComplexField2.Foo);
  REQUIRE(newComplexField.Bar == newComplexField2.Bar);
  verify_map_meta(map);

  map.setValue("Some", pc, 5678);
  
  map.setValue("MoreValues", pc, complexFields2);

  REQUIRE(map.castTo<NewComplexField>("ComplexFields", pc).Foo == "Map is awsome");
  REQUIRE(pc.error == JS::Error::NoError);
  verify_map_meta(map);

  map.setValue("ComplexFields2", pc, false);
  verify_map_meta(map);

  REQUIRE(map.castTo<NewComplexField>("ComplexFields", pc).Foo == "Map is awsome");
  REQUIRE(pc.error == JS::Error::NoError);
}

struct HelloWorld
{
  std::string Hello;
  std::string World;
  JS_OBJ(Hello, World);
};
TEST_CASE("clean_map", "json_struct, map")
{
  JS::Map map;
  JS::ParseContext pc;
  map.setValue("Hello", pc, std::string("world"));
  map.setValue("World", pc, std::string("hello"));
  HelloWorld helloWorld = map.castTo<HelloWorld>(pc);
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(helloWorld.Hello == "world");
  REQUIRE(helloWorld.World == "hello");
}

TEST_CASE("to_clean_map", "json_struct, map")
{
  HelloWorld helloworld;
  helloworld.Hello = "Foo";
  helloworld.World = "Bar";
  JS::Map map;
  JS::ParseContext pc;
  map.setValue(pc, helloworld);
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(map.castTo<std::string>("Hello", pc) == "Foo");
  REQUIRE(pc.error == JS::Error::NoError);
  REQUIRE(map.castTo<std::string>("World", pc) == "Bar"); 
}

//TEST_CASE("fail_to_compile", "json_struct, map")
//{
//  JS::Map map;
//  JS::ParseContext pc;
//  int foo = 44;
//  map.setValue(pc, foo);
//  REQUIRE(pc.error == JS::Error::NoError);
//
//}
} // namespace
