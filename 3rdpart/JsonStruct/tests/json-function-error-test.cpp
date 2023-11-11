#include <json_struct/json_struct.h>

#include "catch2/catch.hpp"

const char json_data[] = R"json(
{
  "execute_one": {
    "prop1": 4,
    "prop2": "Property 2"
  },
  "execute_two": {
    "first_prop": "some string",
    "second_prop": 8
  }
}
)json";

struct ExecuteOneData
{
  int prop1;
  std::string prop2;
  std::string prop3;
  JS_OBJECT(JS_MEMBER(prop1), JS_MEMBER(prop2), JS_MEMBER(prop3));
};
struct ExecuteTwoData
{
  std::string first_prop;
  JS_OBJECT(JS_MEMBER(first_prop));
};
struct ExecuterTwoReturn
{
  std::string string_data;
  int value;
  std::vector<int> values;
  JS_OBJECT(JS_MEMBER(string_data), JS_MEMBER(value), JS_MEMBER(values));
};
struct Executor
{
  void execute_one(const ExecuteOneData &data)
  {
    JS_UNUSED(data);
    execute_one_called = true;
  }
  ExecuterTwoReturn execute_two(const ExecuteTwoData &data)
  {
    JS_UNUSED(data);
    execute_two_called = true;
    ExecuterTwoReturn ret;
    ret.string_data = "Ret data";
    ret.value = 999;
    ret.values = {3, 4, 5, 7, 8};
    return ret;
  }
  bool execute_one_called = false;
  bool execute_two_called = false;

  JS_FUNCTION_CONTAINER(JS_FUNCTION(execute_one), JS_FUNCTION(execute_two));
};

TEST_CASE("test_function_error_simple", "[function][error]")
{
  Executor executor;
  std::string json_out;
  JS::DefaultCallFunctionContext context(json_data, json_out);
  context.callFunctions(executor);

  REQUIRE(context.execution_list.size() == 2);
  REQUIRE(context.execution_list[0].unassigned_required_members.data.size() == 1);
  REQUIRE(context.execution_list[0].unassigned_required_members.data[0] == "prop3");
  REQUIRE(context.execution_list[0].missing_members.data.size() == 0);

  REQUIRE(context.execution_list[1].missing_members.data.size() == 1);
  REQUIRE(context.execution_list[1].missing_members.data[0] == "second_prop");
  REQUIRE(context.execution_list[1].unassigned_required_members.data.size() == 0);
}
