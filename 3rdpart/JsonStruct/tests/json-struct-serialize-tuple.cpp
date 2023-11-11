#include "catch2/catch.hpp"
#include <json_struct/json_struct.h>

#include <stdio.h>

namespace json_struct_serialize_tuple
{

struct Foo
{
  JS::Tuple<int, std::string, float> data;
  JS_OBJECT(JS_MEMBER(data));
};

const char json[] = R"json(
{
  "data": [
    9876,
    "Tuples are cool",
    3.1415
  ]
}
)json";

TEST_CASE("serialize_tuple", "[json_struct][tuple]")
{

  Foo out;
  out.data.get<0>() = 12345;
  out.data.get<1>() = "Hello world";
  out.data.get<2>() = 44.50;
  std::string bar = JS::serializeStruct(out);

  Foo in;
  JS::ParseContext context(json);
  auto error = context.parseTo(in);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(in.data.get<0>() == 9876);
  REQUIRE(std::string("Tuples are cool") == in.data.get<1>());
  REQUIRE(in.data.get<2>() > 3.14);
  REQUIRE(in.data.get<2>() < 3.15);
}

struct TestInt {
  std::tuple<int32_t> member;
  JS_OBJ(member);
};

struct TestBool {
  std::tuple<bool> member;
  JS_OBJ(member);
};

TEST_CASE("bool_tuple", "[json_struct][tuple]")
{
  TestInt tiStruct;
  JS::ParseContext intContext(R"({ "member": [5] })");
  REQUIRE(intContext.parseTo(tiStruct) == JS::Error::NoError);

  TestBool tbStruct;
  JS::ParseContext boolContext(R"({ "member": [true] })");
  REQUIRE(boolContext.parseTo(tbStruct) == JS::Error::NoError);

  std::string serializedTbStruct = JS::serializeStruct(tbStruct);
  REQUIRE(serializedTbStruct.size() != 0);
}

} // namespace json_struct_serialize_tuple
