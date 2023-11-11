#include <string>
#include <json_struct/json_struct.h>

const char car_json[] = R"json(
{
  "type" : "car",
  "wheels" : 4,
  "electric" : true,
  "engine_count" : 4
}
)json";
const char sailboat_json[] = R"json(
{
  "type" : "sailboat",
  "sail_area_m2" : 106.5,
  "swimming_platform": true,
  "cabins_count" : 4
}
)json";

JS_ENUM(VehicleType, car, sailboat)
JS_ENUM_DECLARE_STRING_PARSER(VehicleType)

struct Car
{
  std::string type;
  int wheels;
  bool electric;
  int engine_count;
  JS_OBJ(type, wheels, electric, engine_count);
};

struct Sailboat
{
  std::string type;
  double sail_area_m2;
  bool swimming_platform;
  int cabins_count;
  JS_OBJ(type, sail_area_m2, swimming_platform, cabins_count);
};

void handle_car(Car &car)
{
  (void) car;
  //do stuff with the car
}

void handle_sailboat(Sailboat &sailboat)
{
  (void) sailboat;
  //do stuff with the sailboat
}

void handle_data(const char *data, size_t size)
{
  JS::Map map;
  JS::ParseContext parseContext(data, size, map);
  if (parseContext.error != JS::Error::NoError)
  {
    fprintf(stderr, "Failed to parse Json:\n%s\n", parseContext.makeErrorString().c_str());
    return;
  }
  VehicleType vehicleType = map.castTo<VehicleType>("type", parseContext);
  if (parseContext.error != JS::Error::NoError)
  {
    fprintf(stderr, "Failed to extract type:\n%s\n", parseContext.makeErrorString().c_str());
    return;
  }
  switch (vehicleType)
  {
  case VehicleType::car:
  {
    Car car = map.castTo<Car>(parseContext);
    if (parseContext.error != JS::Error::NoError)
    {
      //error handling 
    }
    handle_car(car);
    break;
  }
  case VehicleType::sailboat:
    Sailboat sailboat;
    map.castToType(parseContext, sailboat);
    if (parseContext.error != JS::Error::NoError)
    {
      //error handling 
    }
    handle_sailboat(sailboat);
    break;
  }

}

int main()
{
  handle_data(car_json, sizeof(car_json));
  handle_data(sailboat_json, sizeof(sailboat_json));
  return 0;
}

