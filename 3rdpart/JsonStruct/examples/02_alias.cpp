#include <string>
#include <json_struct/json_struct.h>

const char json[] = R"json(
{
    "key" : "value",
    "number" : 100,
    "bool" : true
}
)json";

struct JsonData
{
    std::string key;
    int n;
    bool boolean;

    JS_OBJECT(JS_MEMBER(key),
              JS_MEMBER_ALIASES(n, "number", "num", "or_something_else"),
              JS_MEMBER_WITH_NAME(boolean, "bool"));
};

// JS_MEMBER_ALIASES adds additional names to a member
// JS_MEMBER_WITH_NAME replaces the lookup name

int main()
{
    JsonData dataStruct;
    JS::ParseContext parseContext(json);
    if (parseContext.parseTo(dataStruct) != JS::Error::NoError)
    {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        return -1;
    }

    fprintf(stdout, "Key is: %s, number is %d bool is %d\n",
            dataStruct.key.c_str(),
            dataStruct.n,
            dataStruct.boolean);

    return 0;
}

