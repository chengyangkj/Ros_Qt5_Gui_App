#include <string>
#include <json_struct/json_struct.h>

const char json[] = R"json(
{
    "key" : "value",
    "number" : 100,
    "boolean" : true,
    "additional" : "This is defined in the superclass",
    "data" : 1234
}
)json";

struct JsonData
{
    std::string key;
    int number;
    bool boolean;

    JS_OBJ(key, number, boolean);
};

struct SubClass : public JsonData
{
    std::string additional;
    double data;

    JS_OBJ_SUPER(JS_SUPER(JsonData), additional, data);
};

int main()
{
    SubClass dataStruct;
    JS::ParseContext parseContext(json);
    if (parseContext.parseTo(dataStruct) != JS::Error::NoError)
    {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        return -1;
    }

    fprintf(stdout, "Key is: %s, number is %d bool is %d additional is %s data is %f\n",
            dataStruct.key.c_str(),
            dataStruct.number,
            dataStruct.boolean,
            dataStruct.additional.c_str(),
            dataStruct.data);

    return 0;
}

