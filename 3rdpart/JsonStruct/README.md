# **Structurize your JSON**

[![Build status](https://ci.appveyor.com/api/projects/status/mduab0w8u12atfbu?svg=true)](https://ci.appveyor.com/project/jorgen36373/json-struct)

json_struct is a single header only library that parses JSON to C++ structs/classes
and serializing structs/classes to JSON.

It is intended to be used by copying the json_struct.h file from the include
folder into the include path for the project. It is only the json_struct.h file
that is needed to serialize and deserialize json from structures.

It is dependent on some C++11 features and is tested on newer versions of gcc
and clang. It is also tested on VS 2015 and newer.

### Structs

json_struct can parse JSON and automatically populate structures with content
by adding some metadata to the C++ structs.

```json
{
    "One" : 1,
    "Two" : "two",
    "Three" : 3.333
}
```

can be parsed into a structure defined like this:

```c++
struct JsonObject
{
    int One;
    std::string Two;
    double Three;

    JS_OBJ(One, Two, Three);
};
```

or

```c++
struct JsonObject
{
    int One;
    std::string Two;
    double Three;
};
JS_OBJ_EXT(JsonObject, One, Two, Three);
```

Populating the struct would look like this:

```c++
JS::ParseContext context(json_data);
JsonObject obj;
context.parseTo(obj);
```

Serializing the struct to json could be done like this:

```c++
std::string pretty_json = JS::serializeStruct(obj);
// or
std::string compact_json = JS::serializeStruct(obj, JS::SerializerOptions(JS::SerializerOptions::Compact));
```

### Maps

Sometimes the structure of the JSON is dependent on some value in the JSON. Say there is some input JSON that describes a transportation vehicle.
It can looks something like this
```json
{
  "type" : "car",
  "wheels" : 4,
  "electric" : true,
...
}
```
or it could look like this:
```json
{
  "type" : "sailboat",
  "sail_area_m2" : 106.5,
  "swimming_platform": true,
...
}
```

This doesn't fit well with the static nature of json_struct. However, it is
possible to parse the JSON into a map structure, query some child members for
data and then dispatch the conversion into an appropriate type.

```c++
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
```

Here we parse the JSON into a `JS::Map`. This map lets us query if the map
contains a member and it enables us to convert that member into a type. In the example we convert it to the VehicleType:
```c++
  VehicleType vehicleType = map.castTo<VehicleType>("type", parseContext);
```
Then we can inspect the value of the type child and cast the entire object into
the desired type. The `cast` functions have two signatures: `castTo` and
`castToValue`. `castTo` returnes the value type, however, if the object has
already been allocated and just needs to populated then `castToType`.
`castToType` has the added bonus of not needing to specify the template type
since this is deduced by the parameter. Casting the whole object has the same semantics it only misses the "name" parameter:
```c++
    Car car = map.castTo<Car>(parseContext);
    // or
    Sailboat sailboat;
    map.castToType(parseContext, sailboat);
```

### Demystifying the Macros
The JS_OBJ macro adds a static meta object to the struct/class. It does not
affect the semantics or size of the struct/class. It automatically applies
another macro to the member arguments, getting the name and member pointer.
There are other macros that are more verbose, but that gives more flexibility.

The JS_OBJECT macro requires that all the members are passed in a JS_MEMBER
macro. An example of these macros being applied would look like this:
```c++
struct JsonObject
{
    int One;
    std::string Two;
    double Three;

    JS_OBJECT(JS_MEMBER(One)
            , JS_MEMBER(Two)
            , JS_MEMBER(Three));
};
```

This doesn't add any value, but say you want to have a different JSON key for a
member than the name, or maybe you want to add some alias keys, then this
could be done like this:
```c++
struct JsonObject
{
    int One;
    std::string Two;
    double Three;

    JS_OBJECT(JS_MEMBER(One)
            , JS_MEMBER_WITH_NAME(Two, "TheTwo")
            , JS_MEMBER_ALIASES(Three, "TheThree", "the_three"));
};
```

The difference between the _WITH_NAME and _ALIASES macros is that the
_WITH_NAME macro ignores the member name and uses the supplied name, while the
aliases adds a list of names to be checked after the name of the member is
checked.

Its not possible to use the JS_MEMBER macros with the JS_OBJ macro, since then
it tries to apply the JS_MEMBER macro twice on member.

### TypeHandler
For objects the meta information is generated with JS_OBJ and JS_OBJECT macros,
but there might be types that doesn't fit the meta information interface, ie
they are not JSON Object types. Then its possible to define how these specific
classes are serialized and deserialized with the TypeHandler interface.

When the JS::ParseContext tries to parse a type it will look for a template
specialisation of the type:

```c++
namespace JS {
    template<typename T>
    struct TypeHandler;
}
```

There are a number of predefined template specialisations for types such as:

* `std::string`
* `double`
* `float`
* `uint8_t`
* `int16_t`
* `uint16_t`
* `int32_t`
* `uint32_t`
* `int64_t`
* `uint64_t`
* `std::unique_ptr`
* `bool`
* `std::vector`
* `[T]`

Its not often necessary, but when you need to define your own serialization and
deserialization it's done like this:

```c++
namespace JS {
template<>
struct TypeHandler<uint32_t>
{
public:
    static inline Error to(uint32_t &to_type, ParseContext &context)
    {
        char *pointer;
        unsigned long value = strtoul(context.token.value.data, &pointer, 10);
        to_type = static_cast<unsigned int>(value);
        if (context.token.value.data == pointer)
            return Error::FailedToParseInt;
        return Error::NoError;
    }

    static void from(const uint32_t &from_type, Token &token, Serializer &serializer)
    {
        std::string buf = std::to_string(from_type);
        token.value_type = Type::Number;
        token.value.data = buf.data();
        token.value.size = buf.size();
        serializer.write(token);
    }
};
}
```

This gives you complete control of serialization deserialization of a type and it can unfold to a JSON object or array if needed.

For more information checkout the examples at:
https://github.com/jorgen/json_struct/tree/master/examples

and have a look at the more complete unit tests at:
https://github.com/jorgen/json_struct/tree/master/tests
