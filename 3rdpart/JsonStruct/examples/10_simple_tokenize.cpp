#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include <json_struct/json_struct.h>

const char json[] = R"json(
{
    "key" : "value",
    "number" : 100,
    "boolean" : true
}
)json";

int main()
{
    std::string key;
    int number;
    bool boolean;
    JS::Error error;
    JS::Tokenizer tokenizer;
    JS::Token token;
    tokenizer.addData(json);
    error = tokenizer.nextToken(token);
    if (token.value_type == JS::Type::ObjectStart) {
        error = tokenizer.nextToken(token);
        if (error != JS::Error::NoError)
            exit(1);
        key = std::string(token.value.data, token.value.size);
        error = tokenizer.nextToken(token);
        if (error != JS::Error::NoError)
            exit(1);
        char *outpointer;
        number = strtol(token.value.data,
                        &outpointer,
                        10);
        error = tokenizer.nextToken(token);
        if (error != JS::Error::NoError)
            exit(1);
        boolean = std::string(token.value.data, token.value.size) == "true";
        error = tokenizer.nextToken(token);
        if (error != JS::Error::NoError)
            exit(1);
        fprintf(stdout, "Parsed data %s - %d - %d\n", key.c_str(), number, boolean);
    }
    return 0;
}
