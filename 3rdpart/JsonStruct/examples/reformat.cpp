#include <string>
#include <json_struct/json_struct.h>

const char json[] = R"json({"key":"value","number":100,"boolean":true})json";

int main()
{

    std::string pretty_json;
    JS::reformat(json, pretty_json);

    fprintf(stdout, "Json after reformat:\n%s\n", pretty_json.c_str());

    return 0;
}

