#include "catch2/catch.hpp"
#include <json_struct/json_struct.h>
#include <stdio.h>

JS_ENUM(Colors, Red, Green, Blue, Yellow4, Purple)

namespace
{
struct TestEnumParser
{
  Colors colors;

  JS_OBJECT(JS_MEMBER(colors));
};
} // namespace

JS_ENUM_DECLARE_STRING_PARSER(Colors)

namespace
{
const char json[] = R"json({
  "colors": "Green"
})json";

TEST_CASE("check_enum_parser", "[json_struct][enum]")
{
  JS::ParseContext pc(json);
  TestEnumParser ep;
  auto error = pc.parseTo(ep);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(ep.colors == Colors::Green);

  std::string jsonout = JS::serializeStruct(ep);
  REQUIRE(jsonout == json);
}

const char json_number[] = R"json({
  "colors": 2
})json";

TEST_CASE("check_enum_number_parser", "[json_struct][enum]")
{
  JS::ParseContext pc(json_number);
  TestEnumParser ep;
  auto error = pc.parseTo(ep);

  REQUIRE(error == JS::Error::NoError);

  REQUIRE(ep.colors == Colors::Blue);
}

namespace FOO
{
namespace BAR
{
JS_ENUM(Cars, Fiat, VW, BMW, Peugeot, Mazda)
}
} // namespace FOO
namespace One
{
namespace Two
{
struct CarContainer
{
  FOO::BAR::Cars car;

  JS_OBJECT(JS_MEMBER(car));
};
} // namespace Two
} // namespace One

} // namespace

JS_ENUM_NAMESPACE_DECLARE_STRING_PARSER(FOO::BAR, Cars)

namespace
{
const char car_json[] = R"json({
  "car": "BMW"
})json";

TEST_CASE("check_enum_parser_namespace", "[json_struct][enum]")
{
  JS::ParseContext pc(car_json);
  One::Two::CarContainer cc;
  auto error = pc.parseTo(cc);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(cc.car == FOO::BAR::Cars::BMW);

  std::string jsonout = JS::serializeStruct(cc);
  REQUIRE(jsonout == car_json);
}

} // namespace
