#include <json_struct/json_struct.h>
#include "catch2/catch.hpp"

#define JS_STL_UNORDERED_SET
#include <json_struct/json_struct.h>

#define JS_STL_MAP
#define JS_STL_SET
#include <json_struct/json_struct.h>
namespace
{

const char json[] = R"json({
  "unordered_map": {
    "foo": [ 1.0 ],
    "bar": [ 2.0 ]
  },
  "map": {
    "hello": "world",
    "sail": "boat"
  },
  "unordered_set": [9, 8, 7, 6],
  "set": [1.5, 1.6, 1.7, 1.8]
})json";

struct JsonData
{
#ifdef JS_STD_UNORDERED_MAP
  std::unordered_map<std::string, std::vector<double>> unordered_map;
  std::unordered_set<int> unordered_set;
#else
  std::map<std::string, std::vector<double>> unordered_map;
  std::set<int> unordered_set;
#endif
  std::map<std::string, std::string> map;
  std::set<float> set;
  JS_OBJ(unordered_map, map, unordered_set, set);
};

TEST_CASE("unordered_map_complex_value", "json_struct")
{
  JsonData dataStruct;
  JS::ParseContext parseContext(json);
  REQUIRE(parseContext.parseTo(dataStruct) == JS::Error::NoError);

  REQUIRE(dataStruct.unordered_map["bar"].front() == 2.0);
  REQUIRE(dataStruct.map["sail"] == "boat");
  REQUIRE(dataStruct.unordered_set.find(7) != dataStruct.unordered_set.end());
  REQUIRE(dataStruct.set.find(1.6f) != dataStruct.set.end());


  std::vector<double> one;
  one.push_back(1.0);

  std::vector<double> two;
  two.push_back(2.0);
  REQUIRE(dataStruct.unordered_map["foo"] == one);
  REQUIRE(dataStruct.unordered_map["bar"] == two);

  std::string genjson = JS::serializeStruct(dataStruct);

  JsonData dataStruct2;
  REQUIRE(dataStruct2.unordered_map != dataStruct.unordered_map);
  JS::ParseContext parseContext2(genjson);
  REQUIRE(parseContext2.parseTo(dataStruct2) == JS::Error::NoError);

  REQUIRE(dataStruct2.unordered_map == dataStruct.unordered_map);
}
} // namespace
