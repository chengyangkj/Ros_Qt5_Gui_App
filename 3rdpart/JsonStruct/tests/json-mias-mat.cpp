#include "catch2/catch.hpp"
#include <json_struct/json_struct.h>

namespace
{

const char ingredientJsonError[] = R"json(
{
  "id": 666,
  "name": "Polser",
  "base_unit": "g",
  "description": "Fooofooofooo",
  "shop_group_id": 4,
  "energy_per_unit", 3.14,
  "vegetarian": true,
  "vegan": false,
  "grams_per_deciliter": 0.43,
  "grams_per_dimensionless": 0.0,
  "unit": "dl",
  "quantity": 2.0,
  "group_name": "topping",
  "recipe_id": 5,
  "recipe_name": "gryta",
  "use_ingredient_groups": true,
  "portions": 2,
  "portions_unit": "porsjon",
  "shop_group_name": "frukt und grunt",
  "allergens": [
    "fisk",
    "gluten"
  ]
}
)json";

const char shoppingListNameSkipJson[] = R"json(
{
  "ingredients": [
    {
      "id": 123,
      "recipe_name_id_list": {
        "items": []
      },
      "allergens": [],
      "name": "babyleafblader"
    }
  ],
  "userDefinedItems": [
  ],
  "notes": "",
  "fileVersion": 2,
  "sortOrder": 2,
  "name": "Handleliste",
  "dateExplicit": "9. november 2017",
  "timestamp": "2017-11-09 21-52-05",
  "isAutomaticSave": false
}
)json";

struct RecipeNameIdItem
{
  RecipeNameIdItem()
  {
  }

  RecipeNameIdItem(const std::string &recipe_name, int recipe_id)
    : recipe_name(recipe_name)
    , recipe_id(recipe_id)
  {
  }

  std::string recipe_name;
  int recipe_id;

  JS_OBJECT(JS_MEMBER(recipe_name), JS_MEMBER(recipe_id));
};

struct RecipeNameIdList
{
  std::vector<RecipeNameIdItem> items;

  JS_OBJECT(JS_MEMBER(items));
};

struct ShoppingListItemCPP
{
  bool selected;

  JS_OBJECT(JS_MEMBER(selected));
};

struct IngredientCPP : public ShoppingListItemCPP
{
  int id;
  std::string name;
  std::string base_unit;
  std::string description;
  int shop_group_id;
  float energy_per_unit;
  bool vegetarian;
  bool vegan;
  float grams_per_deciliter;
  float grams_per_dimensionless;

  std::string unit;
  float quantity;
  std::string group_name;

  int recipe_id;
  std::string recipe_name;
  bool use_ingredient_groups;
  float portions;
  std::string portions_unit;

  std::string shop_group_name;

  std::vector<std::string> allergens;

  RecipeNameIdList recipe_name_id_list;

  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(ShoppingListItemCPP)), JS_MEMBER(id), JS_MEMBER(name),
                       JS_MEMBER(base_unit), JS_MEMBER(description), JS_MEMBER(shop_group_id),
                       JS_MEMBER(energy_per_unit), JS_MEMBER(vegetarian), JS_MEMBER(vegan),
                       JS_MEMBER(grams_per_deciliter), JS_MEMBER(grams_per_dimensionless), JS_MEMBER(unit),
                       JS_MEMBER(quantity), JS_MEMBER(group_name), JS_MEMBER(recipe_id), JS_MEMBER(recipe_name),
                       JS_MEMBER(use_ingredient_groups), JS_MEMBER(portions), JS_MEMBER(portions_unit),
                       JS_MEMBER(shop_group_name), JS_MEMBER(allergens), JS_MEMBER(recipe_name_id_list));
};

struct UserDefinedItemCPP : public ShoppingListItemCPP
{
  std::string text;

  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(ShoppingListItemCPP)), JS_MEMBER(text));
};

struct ShoppingListFileBase
{
  int fileVersion;
  int sortOrder;
  std::string name;
  std::string dateExplicit;
  std::string timestamp;
  bool isAutomaticSave;

  JS_OBJECT(JS_MEMBER(fileVersion), JS_MEMBER(sortOrder), JS_MEMBER(name), JS_MEMBER(dateExplicit),
            JS_MEMBER(timestamp), JS_MEMBER(isAutomaticSave));
};

struct ShoppingListFileVersion02 : public ShoppingListFileBase
{
  std::vector<IngredientCPP> ingredients;
  std::vector<UserDefinedItemCPP> userDefinedItems;
  std::string notes;

  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(ShoppingListFileBase)), JS_MEMBER(ingredients),
                       JS_MEMBER(userDefinedItems), JS_MEMBER(notes));
};

TEST_CASE("mias_mat_special_unit_test", "[json_struct]")
{
  IngredientCPP ingredient;
  JS::ParseContext pc(ingredientJsonError);
  auto error = pc.parseTo(ingredient);
  REQUIRE(error == JS::Error::ExpectedDelimiter);

  ShoppingListFileBase fileBase;
  JS::ParseContext nameContext(shoppingListNameSkipJson);
  error = nameContext.parseTo(fileBase);
  REQUIRE(error == JS::Error::NoError);
  REQUIRE(fileBase.name == "Handleliste");
}
} // namespace
