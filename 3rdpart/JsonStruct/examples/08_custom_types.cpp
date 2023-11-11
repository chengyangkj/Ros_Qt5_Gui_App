#include <string>
#include <json_struct/json_struct.h>

struct Vec3
{
    double data[3];
};

//We have to define a specialisation of the struct JS::TypeHandler
//This struct handle how types populate c++ types and how c++ types are serialized
namespace JS
{
template<>
struct TypeHandler<Vec3>
{
    static inline Error to(Vec3 &to_type, ParseContext &context)
    {
        //There exists a TypeHandler for T[N] already, so all we have to do is unwrap the
        //data and call the other TypeHandler specialisation
        return TypeHandler<double[3]>::to(to_type.data, context);
    }

    static inline void from(const Vec3 &from_type, Token &token, Serializer &serializer)
    {
        return TypeHandler<double[3]>::from(from_type.data, token, serializer);
    }
};
}

const char json[] = R"json(
{
    "key" : "value",
    "vec" : [ 3, 6, 9]
}
)json";

struct JsonData
{
    std::string key;
    Vec3 vec;

    JS_OBJ(key, vec);
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

    fprintf(stdout, "Key is: %s, number is %f, %f, %f\n",
            dataStruct.key.c_str(),
            dataStruct.vec.data[0],
            dataStruct.vec.data[1],
            dataStruct.vec.data[2]);

    return 0;
}

