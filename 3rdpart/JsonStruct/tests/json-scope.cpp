#include "catch2/catch.hpp"
#include <json_struct/json_struct.h>
#include <stdio.h>

namespace
{

const char json[] = R"json({
  "func1": {
    "arg1": "hello",
    "arg2": "world"
  },
  "func2": {
    "one": [ 1, 2, 3, 4 ],
    "two": true
  },
  "func3": {
    "first":  {
      "advanced": true
    },
    "second": false
  }
})json";

struct Func1Arg
{
  int arg1;
  std::string arg2;
  JS_OBJECT(JS_MEMBER(arg1), JS_MEMBER(arg2));
};

struct Func2Arg
{
  int one[4];
  bool two;
  JS_OBJECT(JS_MEMBER(one), JS_MEMBER(two));
};

struct Func3Arg
{
  int first;
  double second;
  JS_OBJECT(JS_MEMBER(first), JS_MEMBER(second));
};

struct FunctionCont
{
  void func1(const Func1Arg &)
  {
    func1_called = true;
  }

  void func2(const Func2Arg &arg)
  {
    JS_UNUSED(arg);
    func2_called = true;
  }

  void func3(const Func3Arg &arg)
  {
    JS_UNUSED(arg);
    func3_called = true;
  }

  bool func1_called = false;
  bool func2_called = false;
  bool func3_called = false;
  JS_FUNCTION_CONTAINER(JS_FUNCTION(func1), JS_FUNCTION(func2), JS_FUNCTION(func3));
};

TEST_CASE("json_function_scope_test", "[function]")
{
  FunctionCont cont;
  std::string json_out;
  JS::DefaultCallFunctionContext context(json, json_out);
  context.callFunctions(cont);
  REQUIRE(context.error_context.getLatestError() == JS::Error::NoError);
}
} // namespace
