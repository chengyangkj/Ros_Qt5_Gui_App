#define JS_STL_ARRAY 1
#include <json_struct/json_struct.h>

#include "catch2/catch.hpp"

//Attribution:
//The json and the parsable structuress are taken from Stephen Berry's json_performance test. Please see here:
//https://github.com/stephenberry/json_performance

namespace
{
static const char json[] = R"(
{
   "fixed_object": {
      "int_array": [0, 1, 2, 3, 4, 5, 6],
      "float_array": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
      "double_array": [3288398.238, 233e22, 289e-1, 0.928759872, 0.22222848, 0.1, 0.2, 0.3, 0.4]
   },
   "fixed_name_object": {
      "name0": "James",
      "name1": "Abraham",
      "name2": "Susan",
      "name3": "Frank",
      "name4": "Alicia"
   },
   "another_object": {
      "string": "here is some text",
      "another_string": "Hello World",
      "boolean": false,
      "nested_object": {
         "v3s": [[0.12345, 0.23456, 0.001345],
                  [0.3894675, 97.39827, 297.92387],
                  [18.18, 87.289, 2988.298]],
         "id": "298728949872"
      }
   },
   "string_array": ["Cat", "Dog", "Elephant", "Tiger"],
   "string": "Hello world",
   "number": 3.14,
   "boolean": true,
   "another_bool": false
}
)";

struct fixed_object_t
{
   std::vector<int> int_array;
   std::vector<float> float_array;
   std::vector<double> double_array;
};

struct fixed_name_object_t
{
   std::string name0{};
   std::string name1{};
   std::string name2{};
   std::string name3{};
   std::string name4{};
};

struct nested_object_t
{
   std::vector<std::array<double, 3>> v3s{};
   std::string id{};
};

struct another_object_t
{
   std::string string{};
   std::string another_string{};
   bool boolean{};
   nested_object_t nested_object{};
};

struct obj_t
{
   fixed_object_t fixed_object{};
   fixed_name_object_t fixed_name_object{};
   another_object_t another_object{};
   std::vector<std::string> string_array{};
   std::string string{};
   double number{};
   bool boolean{};
   bool another_bool{};
};
}

JS_OBJ_EXT(fixed_object_t, int_array, float_array, double_array);
JS_OBJ_EXT(fixed_name_object_t, name0, name1, name2, name3, name4);
JS_OBJ_EXT(nested_object_t, v3s, id);
JS_OBJ_EXT(another_object_t, string, another_string, boolean, nested_object);
JS_OBJ_EXT(obj_t, fixed_object, fixed_name_object, another_object, string_array, string, number, boolean, another_bool);

TEST_CASE("glaze_json_benchmark", "[performance]")
{

   BENCHMARK("Parse Glaze Json")
   {
     obj_t obj;
     JS::ParseContext context(json);
     context.track_member_assignement_state = false;
     auto error = context.parseTo(obj);
     if (error != JS::Error::NoError)
     {
      fprintf(stderr, "json_struct error: %s\n", context.makeErrorString().c_str());
     }
     return obj;
   };

   BENCHMARK_ADVANCED("Serialize Glaze Json")(Catch::Benchmark::Chronometer meter)
   {
     obj_t obj;
     JS::ParseContext context(json);
     context.track_member_assignement_state = false;
     auto error = context.parseTo(obj);
     if (error != JS::Error::NoError)
     {
       fprintf(stderr, "json_struct error: %s\n", context.makeErrorString().c_str());
     }
     meter.measure([&obj] { return JS::serializeStruct(obj, JS::SerializerOptions(JS::SerializerOptions::Compact));});
   };
}
