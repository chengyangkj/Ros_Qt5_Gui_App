#include "catch2/catch.hpp"
#include <json_struct/json_struct.h>

namespace
{
const char first_child_with_data_json[] = R"json([ [], [],  [
[],
[],
{
  "this has a member": true
},
[]
], [], []])json";

TEST_CASE("find_first_child_with_data", "[json_struct][meta]")
{
  JS::ParseContext pc(first_child_with_data_json);
  JS::JsonTokens tokens;
  auto error = pc.parseTo(tokens);
  REQUIRE(error == JS::Error::NoError);
  std::vector<JS::JsonMeta> meta = JS::metaForTokens(tokens);
  size_t first_child = JS::Internal::findFirstChildWithData(meta, 0);
  REQUIRE(first_child == 2);
}
const char first_child_with_data_json_last[] = R"json([ [], [], [],  [
[],
[],
{
  "this has a member": true
},
[]
]])json";

TEST_CASE("find_first_child_with_data_last", "[json_struct][meta]")
{
  JS::ParseContext pc(first_child_with_data_json_last);
  JS::JsonTokens tokens;
  auto error = pc.parseTo(tokens);
  REQUIRE(error == JS::Error::NoError);
  std::vector<JS::JsonMeta> meta = JS::metaForTokens(tokens);
  size_t first_child = JS::Internal::findFirstChildWithData(meta, 0);
  REQUIRE(first_child == 3);
}
} // namespace
