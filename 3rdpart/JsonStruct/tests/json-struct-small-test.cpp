#include <json_struct/json_struct.h>
#include "assert.h"

struct SmallStruct
{
  int a;
  float b;

  JS_OBJECT(
    JS_MEMBER(a),
    JS_MEMBER(b)
  );
};

const char json[] = R"json(
{
  "a": 1,
  "b": 2.2
}
)json";

int main()
{
  JS::ParseContext context(json);
    SmallStruct data;
    context.parseTo(data);
  JS_ASSERT(data.a == 1);
  JS_ASSERT(data.b > 2.199 && data.b < 2.201);
  return 0;
}
