#include <json_struct/json_struct.h>
#ifdef JS_STD_OPTIONAL
#include <optional>
#endif
#include "catch2/catch.hpp"

namespace
{
struct SmallStructWithoutOptional
{
  int a;
  float b = 2.2f;
  std::string d;

  JS_OBJ(a, b, d);
};

#ifdef JS_STD_OPTIONAL
struct SmallStructStd
{
  int a;
  std::optional<float> b = 2.2f;
  std::optional<std::string> c;
  std::optional<std::string> d;

  JS_OBJ(a, b, c, d);
};
#endif

const char json[] = R"json(
{
  "a": 1,
  "b": 2.2,
  "c": "hello world"
}
)json";

TEST_CASE("test_optional", "[json_struct]")
{
  {
    JS::ParseContext context(json);
    context.allow_unasigned_required_members = false;
    SmallStructWithoutOptional data;
    auto error = context.parseTo(data);
    REQUIRE(error != JS::Error::NoError);
  }
#ifdef JS_STD_OPTIONAL
  {
    JS::ParseContext context(json);
    context.allow_unasigned_required_members = false;
    SmallStructStd data;
    auto error = context.parseTo(data);
    REQUIRE(error == JS::Error::NoError);
    REQUIRE(context.error == JS::Error::NoError);
    REQUIRE(data.a == 1);
    REQUIRE(data.b.value() > 2.199);
    REQUIRE(data.b.value() < 2.201);
    REQUIRE(data.c.value() == "hello world");
  }
#endif
}
} // namespace
