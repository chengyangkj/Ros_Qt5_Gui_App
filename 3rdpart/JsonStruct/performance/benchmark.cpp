#include "assert.h"
#include "generated.json.h"
#include <json_struct/json_struct.h>
#include <chrono>

#include "catch2/catch.hpp"

#include "include/rapidjson/document.h"
#include "include/simdjson/simdjson.h"
#include "include/nlohmann/json.hpp"

TEST_CASE("Benchmarks", "[performance]")
{
  BENCHMARK("Tokenizer_SmallObject")
  {
    JS::Tokenizer tokenizer;
    SmallPerson smallPerson;
    tokenizer.addData(generatedJsonObject, sizeof(generatedJsonObject)-1);

    JS::Token token;
    JS::Error error = JS::Error::NoError;
    int object_count = 0;
    do
    {
      error = tokenizer.nextToken(token);
      if (token.value_type == JS::Type::ObjectStart)
      {
        object_count++;
      }
      else if (token.value_type == JS::Type::ObjectEnd)
      {
        object_count--;
      }
      else if (object_count == 1 && token.value_type == JS::Type::String && token.name.size == 4 &&
               memcmp(token.name.data, "name", 4) == 0)
      {
        smallPerson.name = std::string(token.value.data, token.value.size);
      }
    } while (object_count > 0 && error == JS::Error::NoError);

    if (error != JS::Error::NoError)
      fprintf(stderr, "Failed to parse document\n");
    return smallPerson;
  };
  BENCHMARK("JsonStruct_SmallStruct_Object")
  {
    JS::ParseContext context(generatedJsonObject, sizeof(generatedJsonObject)-1);
    SmallPerson person;
    context.parseTo(person);
    return person;
  };

  BENCHMARK("RapidJson_SmallStruct_Object")
  {
    rapidjson::Document d;
    d.Parse(generatedJsonObject);
    SmallPerson smallPerson;
    smallPerson.name = d["name"].GetString();
    return smallPerson;
  };

  BENCHMARK("RapidJson_WithSizeOfJson_SmallStruct_Object")
  {
    rapidjson::Document d;
    d.Parse(generatedJsonObject, sizeof(generatedJsonObject) -1);
    SmallPerson smallPerson;
    smallPerson.name = d["name"].GetString();
    return smallPerson;
  };
  
  BENCHMARK("SimdJson_SmallStruct_Object")
  {
    simdjson::dom::parser parser;
    const char *json = generatedJsonObject;
    SmallPerson smallPerson;
    simdjson::dom::element document = parser.parse(json, sizeof(generatedJsonObject)-1);
    smallPerson.name = std::string(document["name"].get_string().first);
    return smallPerson;
  };
  
  BENCHMARK("Nlohmann_JsonForModernC++_Dom_SmallStruct_Object")
  {
    auto dom = nlohmann::json::parse(generatedJsonObject, generatedJsonObject + sizeof(generatedJsonObject) - 1);
    int max_size = int(dom.size());
    SmallPerson smallPerson;
    smallPerson.name = dom["name"];
    return smallPerson;
  };
  
  BENCHMARK("Nlohmann_JsonForModernC++_SmallStructObject")
  {
    auto d = nlohmann::json::parse(generatedJsonObject, generatedJsonObject + sizeof(generatedJsonObject) - 1);
    auto person = d.get<SmallPerson>();
    return person;
  };


  BENCHMARK("JsonStruct_FullStruct_Object")
  {
    JS::ParseContext context(generatedJsonObject, sizeof(generatedJsonObject)-1);
    JPerson person;
    context.parseTo(person);
    return person;
  };

  BENCHMARK("RapidJson_FullStruct_Object")
  {
    rapidjson::Document d;
    d.Parse(generatedJsonObject);
    JPerson person;
    person._id = d["_id"].GetString();
    person.index = d["index"].GetInt();
    person.guid = d["guid"].GetString();
    person.isActive = d["isActive"].GetBool();
    person.balance = d["balance"].GetString();
    person.picture = d["picture"].GetString();
    person.age = d["age"].GetInt();
    person.eyeColor = d["eyeColor"].GetString();
    person.name = d["name"].GetString();
    person.gender = d["gender"].GetString();
    person.company = d["company"].GetString();
    person.email = d["email"].GetString();
    person.phone = d["phone"].GetString();
    person.address = d["address"].GetString();
    person.about = d["about"].GetString();
    person.registered = d["registered"].GetString();
    person.latitude = d["latitude"].GetFloat();
    person.longitude = d["longitude"].GetFloat();
    auto &tags = d["tags"];
    person.tags.reserve(tags.Size());
    for (auto it = tags.Begin(); it != tags.End(); ++it)
    {
      person.tags.push_back(it->GetString());
    }
    auto &friends = d["friends"];
    person.friends.reserve(friends.Size());
    for (auto it = friends.Begin(); it != friends.End(); ++it)
    {
      Friends f;
      f.id = it->GetObject()["id"].GetInt();
      f.name = it->GetObject()["name"].GetString();
      person.friends.push_back(std::move(f));
    }
    person.greeting = d["greeting"].GetString();
    person.favoriteFruit = d["favoriteFruit"].GetString();
    return person;
  };

  BENCHMARK("SimdJson_FullStruct_Object")
  {
    simdjson::dom::parser parser;
    const char *json = generatedJsonObject;
    JPerson person;
    simdjson::dom::element d = parser.parse(json, sizeof(generatedJsonObject)-1);
    person._id = std::string(d["_id"].get_string().first);
    person.index = int(d["index"].get_int64());
    person.guid = std::string(d["guid"].get_string().first);
    person.isActive = d["isActive"].get_bool();
    person.balance = std::string(d["balance"].get_string().first);
    person.picture = std::string(d["picture"].get_string().first);
    person.age = int(d["age"].get_int64());
    person.eyeColor = std::string(d["eyeColor"].get_string().first);
    person.name = std::string(d["name"].get_string().first);
    person.gender = std::string(d["gender"].get_string().first);
    person.company = std::string(d["company"].get_string().first);
    person.email = std::string(d["email"].get_string().first);
    person.phone = std::string(d["phone"].get_string().first);
    person.address = std::string(d["address"].get_string().first);
    person.about = std::string(d["about"].get_string().first);
    person.registered = std::string(d["registered"].get_string().first);
    person.latitude = float(d["latitude"].get_double());
    person.longitude = float(d["longitude"].get_double());
    auto tags = d["tags"].get_array().first;
    person.tags.reserve(tags.size());
    for (auto it = tags.begin(); it != tags.end(); ++it)
    {
      person.tags.emplace_back((*it).get_string().first);
    }
    auto friends = d["friends"].get_array().first;
    person.friends.reserve(friends.size());
    for (auto it = friends.begin(); it != friends.end(); ++it)
    {
      Friends f;
      f.id = int((*it)["id"].get_int64().first);
      f.name = std::string((*it)["name"].get_string().first);
      person.friends.push_back(std::move(f));
    }
    person.greeting = std::string(d["greeting"].get_string().first);
    person.favoriteFruit = std::string(d["favoriteFruit"].get_string().first);
    return person;
  };

  BENCHMARK("Nlohmann_JsonForModernC++_FullStruct_Object")
  {
    auto d = nlohmann::json::parse(generatedJsonObject, generatedJsonObject + sizeof(generatedJsonObject) - 1);
    auto person = d.get<JPerson>();
    return person;
  };

  BENCHMARK("Tokenizer_SmallStruct_Array")
  {
    JS::Tokenizer tokenizer;
    SmallPerson smallPerson;
    tokenizer.addData(generatedJsonArray, sizeof(generatedJsonArray)-1);

    JS::Token token;
    JS::Error error = JS::Error::NoError;
    int array_size = 0;
    int max_size = 0;
    do
    {
      error = tokenizer.nextToken(token);
      if (token.value_type == JS::Type::ArrayStart)
      {
        array_size++;
      }
      else if (token.value_type == JS::Type::ArrayEnd)
      {
        array_size--;
      }
      else if (token.value_type == JS::Type::ObjectStart && array_size == 1)
      {
        max_size++;
      }
      else if (max_size == 2 && array_size == 1 && token.value_type == JS::Type::String && token.name.size == 4 &&
               memcmp(token.name.data, "name", 4) == 0)
      {
        smallPerson.name = std::string(token.value.data, token.value.size);
      }
    } while (array_size > 0 && error == JS::Error::NoError);

    if (error != JS::Error::NoError)
      fprintf(stderr, "Failed to parse document\n");
    return smallPerson;
  };

  BENCHMARK("JsonStruct_SmallStruct_Array")
  {
    JS::ParseContext context(generatedJsonArray, sizeof(generatedJsonArray)-1);
    std::vector<SmallPerson> people;
    context.parseTo(people);
    return people;
  };
  BENCHMARK("RapidJson_SmallStruct_Array")
  {
    rapidjson::Document d;
    d.Parse(generatedJsonArray);
    SmallPerson smallPerson;
    smallPerson.name = d[1]["name"].GetString();
    return smallPerson;
  };
  BENCHMARK("RapidJson_WithSizeOfJson_SmallStruct_Array")
  {
    rapidjson::Document d;
    d.Parse(generatedJsonArray, sizeof(generatedJsonArray) -1);
    SmallPerson smallPerson;
    smallPerson.name = d[1]["name"].GetString();
    return smallPerson;
  };
  
  BENCHMARK("SimdJson_SmallStruct_Array")
  {
    simdjson::dom::parser parser;
    const char *json = generatedJsonArray;
    SmallPerson smallPerson;
    simdjson::dom::element document = parser.parse(json, sizeof(generatedJsonArray)-1);
    smallPerson.name = std::string(document.at(1)["name"].get_string().first);
    return smallPerson;
  };

  BENCHMARK("Nlohmann_JsonForModernC++_DOM_SmallStruct_Array")
  {
    auto dom = nlohmann::json::parse(generatedJsonArray, generatedJsonArray + sizeof(generatedJsonArray) - 1);
    int max_size = int(dom.size());
    SmallPerson smallPerson;
    smallPerson.name = dom.at(1)["name"];
    return smallPerson;
  };
 
  BENCHMARK("Nlohmann_JsonForModernC++_SmallStruct_Array")
  {
    auto d = nlohmann::json::parse(generatedJsonArray, generatedJsonArray + sizeof(generatedJsonArray) - 1);
    auto people = d.get<std::vector<SmallPerson>>();
    return people;
  };
  
  BENCHMARK("JsonStruct_FullStruct_Array")
  {
    JS::ParseContext context(generatedJsonArray, sizeof(generatedJsonArray)-1);
    std::vector<JPerson> people;
    context.parseTo(people);
    return people;
  };

  BENCHMARK("RapidJson_FullStruct_Array")
  {
    rapidjson::Document d;
    d.Parse(generatedJsonArray);
    JPerson person;
    person._id = d[1]["_id"].GetString();
    person.index = d[1]["index"].GetInt();
    person.guid = d[1]["guid"].GetString();
    person.isActive = d[1]["isActive"].GetBool();
    person.balance = d[1]["balance"].GetString();
    person.picture = d[1]["picture"].GetString();
    person.age = d[1]["age"].GetInt();
    person.eyeColor = d[1]["eyeColor"].GetString();
    person.name = d[1]["name"].GetString();
    person.gender = d[1]["gender"].GetString();
    person.company = d[1]["company"].GetString();
    person.email = d[1]["email"].GetString();
    person.phone = d[1]["phone"].GetString();
    person.address = d[1]["address"].GetString();
    person.about = d[1]["about"].GetString();
    person.registered = d[1]["registered"].GetString();
    person.latitude = d[1]["latitude"].GetFloat();
    person.longitude = d[1]["longitude"].GetFloat();
    auto &tags = d[1]["tags"];
    person.tags.reserve(tags.Size());
    for (auto it = tags.Begin(); it != tags.End(); ++it)
    {
      person.tags.push_back(it->GetString());
    }
    auto &friends = d[1]["friends"];
    person.friends.reserve(friends.Size());
    for (auto it = friends.Begin(); it != friends.End(); ++it)
    {
      Friends f;
      f.id = it->GetObject()["id"].GetInt();
      f.name = it->GetObject()["name"].GetString();
      person.friends.push_back(std::move(f));
    }
    person.greeting = d[1]["greeting"].GetString();
    person.favoriteFruit = d[1]["favoriteFruit"].GetString();
    return person;
  };

  BENCHMARK("SimdJson_FullStruct_Array")
  {
    simdjson::dom::parser parser;
    const char *json = generatedJsonArray;
    JPerson person;
    simdjson::dom::element d = parser.parse(json, sizeof(generatedJsonArray)-1);
    person._id = std::string(d.at(1)["_id"].get_string().first);
    person.index = int(d.at(1)["index"].get_int64());
    person.guid = std::string(d.at(1)["guid"].get_string().first);
    person.isActive = d.at(1)["isActive"].get_bool();
    person.balance = std::string(d.at(1)["balance"].get_string().first);
    person.picture = std::string(d.at(1)["picture"].get_string().first);
    person.age = int(d.at(1)["age"].get_int64());
    person.eyeColor = std::string(d.at(1)["eyeColor"].get_string().first);
    person.name = std::string(d.at(1)["name"].get_string().first);
    person.gender = std::string(d.at(1)["gender"].get_string().first);
    person.company = std::string(d.at(1)["company"].get_string().first);
    person.email = std::string(d.at(1)["email"].get_string().first);
    person.phone = std::string(d.at(1)["phone"].get_string().first);
    person.address = std::string(d.at(1)["address"].get_string().first);
    person.about = std::string(d.at(1)["about"].get_string().first);
    person.registered = std::string(d.at(1)["registered"].get_string().first);
    person.latitude = float(d.at(1)["latitude"].get_double());
    person.longitude = float(d.at(1)["longitude"].get_double());
    auto tags = d.at(1)["tags"].get_array().first;
    person.tags.reserve(tags.size());
    for (auto it = tags.begin(); it != tags.end(); ++it)
    {
      person.tags.emplace_back((*it).get_string().first);
    }
    auto friends = d.at(1)["friends"].get_array().first;
    person.friends.reserve(friends.size());
    for (auto it = friends.begin(); it != friends.end(); ++it)
    {
      Friends f;
      f.id = int((*it)["id"].get_int64().first);
      f.name = std::string((*it)["name"].get_string().first);
      person.friends.push_back(std::move(f));
    }
    person.greeting = std::string(d.at(1)["greeting"].get_string().first);
    person.favoriteFruit = std::string(d.at(1)["favoriteFruit"].get_string().first);
    return person;
  };

  BENCHMARK("Nlohmann_JsonForModernC++_FullStruct_Array")
  {
    auto d = nlohmann::json::parse(generatedJsonArray, generatedJsonArray + sizeof(generatedJsonArray) - 1);
    auto people = d.get<std::vector<JPerson>>();
    return people;
  };

}

