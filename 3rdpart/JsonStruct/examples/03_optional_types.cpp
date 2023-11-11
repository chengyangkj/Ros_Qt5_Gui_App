//Please see example 06_stdtypes for std::optional support
#include <string>
#include <json_struct/json_struct.h>

const char json[] = R"json(
{
    "key" : "value",
    "number" : 100,
    "boolean" : true,
    "opt_checked" : 1.5,
    "member_not_in_c++" : "some_value"
}
)json";

struct JsonData
{
    std::string key;
    int number;
    bool boolean;
    //sizeof(JS::Optional<double>) == sizeof(double) however if you want to see
    //if it has been modified there is the JS::OptionalChecked. It has an "assigned" member
    //allowing the user to inspect if the type has been assigned
    JS::Optional<double> opt = 32.5;
    JS::OptionalChecked<double> opt_checked = 100;

    std::string member_not_in_json;

    JS_OBJ(key, number, boolean, opt, opt_checked, member_not_in_json);
};

int main()
{
    JsonData dataStruct;
    JS::ParseContext parseContext(json);

    //default values, but can be set to false to make parseTo return an error value
    parseContext.allow_missing_members = true;
    parseContext.allow_unasigned_required_members = true;

    //JS::ParseContext has the member
    if (parseContext.parseTo(dataStruct) != JS::Error::NoError)
    {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        return -1;
    }

    //missing C++ struct members found in the json can be retrieved with
    for (std::string &member : parseContext.missing_members)
    {
        fprintf(stderr, "missing member: %s\n", member.c_str());
    }

    //missing JSON member required by non optional C++ members can be retrieved with
    for (std::string &member : parseContext.unassigned_required_members)
    {
        fprintf(stderr, "unassigned required c++ member %s\n", member.c_str());
    }

    fprintf(stdout, "Key is: %s, number is %d bool is %d"
            "optional value is %f optional checked %d <-> %f\n",
            dataStruct.key.c_str(),
            dataStruct.number,
            dataStruct.boolean,
            dataStruct.opt.data,
            dataStruct.opt_checked.assigned,
            dataStruct.opt_checked.data);

    return 0;
}

