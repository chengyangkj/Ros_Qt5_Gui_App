#include <string>
#include <json_struct/json_struct.h>
#include <cinttypes>
//these two have to be ifdef guarded becuase JS support compiler versions where
//they are not implemented, hence if you use unordered_map or optional in your code there is no need for the guards.
#ifdef JS_STD_UNORDERED_MAP
#include <unordered_map>
#endif
#ifdef JS_STD_OPTIONAL
#include <optional>
#endif


const char json[] = R"json(
{
    "vector" : [ 9,7,5,3,1 ],
    "string" : "hello world",
    "tuple" : [ "hello", 32.0, { "member" : 1 } ],
    "unordered_map" : {
        "foo" : 1,
        "bar" : 2
    },
    "optional" : "optioal string value",
    "uint8_number" : 234,
    "uint16_number" : 1234,
    "uint32_number" : 234567,
    "uint64_number" : 5000000000
}
)json";

struct SubType
{
    int member;
    JS_OBJ(member);
};

struct JsonData
{
    std::vector<int> vector;
    std::string string;
    std::tuple<std::string, float, SubType> tuple;
#ifdef JS_STD_UNORDERED_MAP
    std::unordered_map<std::string, double> unordered_map;
#else
    JS::JsonObject unordered_map;
#endif
#ifdef JS_STD_OPTIONAL
    std::optional<std::string> optional;
#else
    JS::Optional<std::string> optional;
#endif

    uint8_t uint8_number;
    uint16_t uint16_number;
    uint32_t uint32_number;
    uint64_t uint64_number;

    JS_OBJ(vector,
           string,
           tuple,
           unordered_map,
           optional,
           uint8_number,
           uint16_number,
           uint32_number,
           uint64_number);
};

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

    for(int i = 0; i < int(dataStruct.vector.size()); i++)
        fprintf(stderr, "vector: %d %d\n", i, dataStruct.vector[i]);
    fprintf(stderr, "string %s\n", dataStruct.string.c_str());
    fprintf(stderr, "tuple:\n");
    fprintf(stderr, "\t%s\n", std::get<0>(dataStruct.tuple).c_str());
    fprintf(stderr, "\t%f\n", std::get<1>(dataStruct.tuple));
    fprintf(stderr, "\tmember: %d\n", std::get<2>(dataStruct.tuple).member);
#ifdef JS_STD_UNORDERED_MAP
    fprintf(stderr, "unordered_map:\n");
    for (auto it : dataStruct.unordered_map)
        fprintf(stderr, "\t%s : %f\n", it.first.c_str(), it.second);
#endif
#ifdef JS_STD_OPTIONAL
    fprintf(stderr, "%s\n", dataStruct.optional.value());
#endif
    fprintf(stderr, "uint8_number %" PRIu8 "\n", dataStruct.uint8_number);
    fprintf(stderr, "uint16_number %" PRIu16 "\n", dataStruct.uint16_number);
    fprintf(stderr, "uint32_number %" PRIu32 "\n", dataStruct.uint32_number);
    fprintf(stderr, "uint64_number %" PRIu64 "\n", dataStruct.uint64_number);
    return 0;
}

