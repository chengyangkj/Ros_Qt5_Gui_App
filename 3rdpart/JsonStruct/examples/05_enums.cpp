#include <stdio.h>
#include <json_struct/json_struct.h>

JS_ENUM(Color, Red , Green , Blue, Yellow4 , Purple )

struct ColorData
{
    Color color;

    JS_OBJ(color);
};
JS_ENUM_DECLARE_STRING_PARSER(Color)

const char json[] = R"json({
    "color" : "Green"
})json";

int main()
{
    ColorData dataStruct;
    JS::ParseContext parseContext(json);
    if (parseContext.parseTo(dataStruct) != JS::Error::NoError)
    {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        return -1;
    }

    fprintf(stdout, "Color data is: %d, enum green has value %d\n",
            (int)dataStruct.color,
            (int)Color::Green);

    return 0;
}

