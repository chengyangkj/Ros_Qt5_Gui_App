#define JS_STL_UNORDERED_SET
#include <json_struct/json_struct.h>
#include "catch2/catch.hpp"

namespace JS
{
template<typename T>
struct TypeHandler<std::unordered_map<int, T>>
{
  static inline Error to(std::unordered_map<int,T> &to_type, ParseContext &context)
  {
    if (context.token.value_type != Type::ObjectStart)
    {
      return JS::Error::ExpectedObjectStart;
    }

    Error error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;

    const char *pointer = nullptr;
    int key = 0;

    while (context.token.value_type != Type::ObjectEnd)
    {
      auto parse_error = Internal::ft::integer::to_integer(context.token.name.data, context.token.name.size, key, pointer);
      if (parse_error != Internal::ft::parse_string_error::ok || context.token.name.data == pointer)
        return Error::FailedToParseInt;

      T value;
      error = TypeHandler<T>::to(value, context);
      to_type[std::move(key)] = std::move(value);
      if (error != JS::Error::NoError)
        return error;
      error = context.nextToken();
    }

    return error;
  }
  static inline void from(const std::unordered_map<int, T> &from_type, Token &token, Serializer &serializer)
  {
    token.value_type = Type::ObjectStart;
    token.value = DataRef("{");
    serializer.write(token);
    char buf[40];
    int digits_truncated;
    for (auto it = from_type.begin(); it != from_type.end(); ++it)
    {
      int size = Internal::ft::integer::to_buffer(it->first, buf, sizeof(buf), &digits_truncated);
      if (size <= 0 || digits_truncated)
        return;
      token.name = DataRef(buf, size);
      token.name_type = Type::String;
      TypeHandler<T>::from(it->second, token, serializer);
    }
    token.name.size = 0;
    token.name.data = "";
    token.name_type = Type::String;
    token.value_type = Type::ObjectEnd;
    token.value = DataRef("}");
    serializer.write(token);
  }
};
}

namespace
{
const char json[] = R"json(
{
    "1": {
        "2": {
            "Foo": {
                "3": {
                    "4": {
                        "Bar": {
                            "1": "1",
                            "2": "2",
                            "3": "3"
                        },
                        "Hello": "World",
                        "World": "Hello",
                        "Value": 10.0
                    }
                },
                "5": {
                    "6": {
                        "Bar": {
                            "1": "1",
                            "2": "2",
                            "3": "3"
                        },
                        "Hello": "World",
                        "World": "Hello",
                        "Value": 10.0
                    }
                }
            },
            "Baz": "FooBarBaz"
        },
        "7": {
            "Foo": {
                "8": {
                    "9": {
                        "Bar": null,
                        "Hello": "World",
                        "World": "Hello",
                        "Value": 11.0
                    }
                }
            },
            "Baz": "FooBarBaz"
        }
    }
}
)json";

struct HelloWorldContainer
{
  JS::Nullable<std::unordered_map<int, std::string>> Bar;
  std::string Hello;
  std::string World;
  float Value;
  JS_OBJ(Bar, Hello, World, Value);
};

struct ParentContainer
{
  std::unordered_map<int, std::unordered_map<int, HelloWorldContainer>> Foo;
  std::string Baz;
  JS_OBJ(Foo, Baz);
};

TEST_CASE("map_typehandler", "json_struct")
{
  std::unordered_map<int, std::unordered_map<int, ParentContainer>> obj;
  JS::ParseContext pc(json);
  auto error = pc.parseTo(obj);
  if (error != JS::Error::NoError)
  {
    auto errorStr = pc.makeErrorString();
    fprintf(stderr, "%s\n", errorStr.c_str());
  }
  for (auto &unassigned : pc.unassigned_required_members)
    fprintf(stderr, "Unnassigned: %s\n", unassigned.c_str());
  for (auto &missing : pc.missing_members)
    fprintf(stderr, "Missing: %s\n", missing.c_str());
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(pc.unassigned_required_members.size() == 0);

  REQUIRE(pc.missing_members.size() == 0);

}
}
