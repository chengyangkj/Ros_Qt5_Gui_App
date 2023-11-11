//!!!!!
//THIS IS A VERY ADVANCED SAMPLE
//PLEASE LOOK AT THE OTHER SAMPLES FIRST
#include <string>
#include <json_struct/json_struct.h>

const char json[] = R"json(
{
    "function_a" : "Some text",
    "function_b" : {
        "paramA" : 123.4,
        "paramB" : "some string parameter"
    },
    "function_c" : {
        "this_function" : 3,
        "can_fail_at_runtime" : true
    },
    "function_d" : 567
}
)json";

struct FunctionBArguments
{
    float paramA;
    std::string paramB;
    JS_OBJ(paramA, paramB);
};

struct FunctionBReturn
{
    float functionBReturnA;
    std::string functionBReturnB;
    double functionBReturnC[3];

    JS_OBJ(functionBReturnA, functionBReturnB, functionBReturnC);
};

struct FunctionCArguments
{
    int this_function;
    bool can_fail_at_runtime;

    JS_OBJ(this_function, can_fail_at_runtime);
};

struct FunctionCReturn
{
    int this_return = 0;
    int type_will_not = 0;
    int be_serialized = 0;
    int on_failure = 0;
    JS_OBJ(this_return, type_will_not, be_serialized, on_failure);
};

struct JsonFunctions
{
    void function_a(const std::string &str)
    {
        fprintf(stderr, "Function a was called with %s\n", str.c_str());
    }

    FunctionBReturn function_b(const FunctionBArguments &arg)
    {
        fprintf(stderr, "Function b was called with %f - %s\n", arg.paramA, arg.paramB.c_str());
        FunctionBReturn ret;
        ret.functionBReturnA = arg.paramA;
        ret.functionBReturnB = "This is the return object";
        ret.functionBReturnC[0] = 3.3;
        ret.functionBReturnC[1] = 4.4;
        ret.functionBReturnC[2] = 5.5;
        return ret;
    }

    FunctionCReturn function_c(const FunctionCArguments &arg, JS::CallFunctionErrorContext &context)
    {
        (void)arg;
        fprintf(stderr, "Function c was called and its going to fail miserably\n");
        FunctionCReturn ret;
        context.setError(JS::Error::UserDefinedErrors, "Making the error"
                         " context have failure marked so that it will not"
                         " serialize the return type");
        return ret;
    }

    bool function_d(int arg)
    {
        fprintf(stderr, "Function d shows that just simple types can be used - %d\n", arg);
        return arg;
    }
    JS_FUNC_OBJ(function_a, function_b, function_c, function_d);
};

int main()
{
    JsonFunctions functionObject;
    std::string output;
    JS::DefaultCallFunctionContext callFunctionContext(json, output);
    callFunctionContext.stop_execute_on_fail = false;
    if (callFunctionContext.callFunctions(functionObject) != JS::Error::NoError)
    {
        std::string errorStr = callFunctionContext.parse_context.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
    }

    for (auto &executed : callFunctionContext.execution_list)
    {
        std::string executionStateJson = JS::serializeStruct(executed);
        fprintf(stderr, "###\n%s\n", executionStateJson.c_str());
    }

    fprintf(stderr, "This is the result:\n%s\n", output.c_str());
    return 0;
}


