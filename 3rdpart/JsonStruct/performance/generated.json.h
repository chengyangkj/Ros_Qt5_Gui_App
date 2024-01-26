#pragma once
#include <json_struct/json_struct.h>
#include "include/nlohmann/json.hpp"

struct Friends {
  int id;
  std::string name;
};
JS_OBJ_EXT(Friends, id, name);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Friends, id, name);

struct JPerson {
  std::string _id;
  int index;
  std::string guid;
  bool isActive;
  std::string balance;
  std::string picture;
  int age;
  std::string eyeColor;
  std::string name;
  std::string gender;
  std::string company;
  std::string email;
  std::string phone;
  std::string address;
  std::string about;
  std::string registered;
  float latitude;
  float longitude;
  std::vector<std::string> tags;
  std::vector<Friends> friends;
  std::string greeting;
  std::string favoriteFruit;
};
JS_OBJ_EXT(JPerson, _id, index, guid, isActive, balance, picture, age, eyeColor, name, gender, company, email, phone,
           address, about, registered, latitude, longitude, tags, friends, greeting, favoriteFruit);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(JPerson, _id, index, guid, isActive, balance, picture, age, eyeColor, name, gender,
                                   company, email, phone, address, about, registered, latitude, longitude, tags,
                                   friends, greeting, favoriteFruit);

struct SmallPerson {
  std::string name;
};
JS_OBJ_EXT(SmallPerson, name);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SmallPerson, name);

const char generatedJsonObject[] = R"gen_test_json({
    "_id": "59404dbcaf79371edd5f3324",
    "index": 0,
    "guid": "d9320115-a89d-4159-8416-6fcde98822b9",
    "isActive": true,
    "balance": "$2,489.22",
    "picture": "http://placehold.it/32x32",
    "age": 34,
    "eyeColor": "blue",
    "name": "Lana Finch",
    "gender": "female",
    "company": "PUSHCART",
    "email": "lanafinch@pushcart.com",
    "phone": "+1 (886) 407-2788",
    "address": "847 Albee Square, Whitestone, Rhode Island, 6120",
    "about": "Quis do non ut incididunt. Quis consectetur pariatur Lorem duis reprehenderit do. Occaecat in est dolore ipsum consequat in. Labore sint commodo officia culpa anim reprehenderit officia. Consequat duis nulla commodo voluptate deserunt eu sunt.\r\n",
    "registered": "2015-09-08T01:06:09 -02:00",
    "latitude": 52.540694,
    "longitude": -170.744891,
    "tags": [
      "proident",
      "esse",
      "reprehenderit",
      "Lorem",
      "mollit",
      "cupidatat",
      "cupidatat"
    ],
    "friends": [
      {
        "id": 0,
        "name": "Jarvis Oconnor"
      },
      {
        "id": 1,
        "name": "Enid Calderon"
      },
      {
        "id": 2,
        "name": "Valencia Delacruz"
      }
    ],
    "greeting": "Hello, Lana Finch! You have 6 unread messages.",
    "favoriteFruit": "strawberry"
  }
)gen_test_json";

const char generatedJsonArray[] = R"gen_test_json(
[
  {
    "_id": "59404dbcaf79371edd5f3324",
    "index": 0,
    "guid": "d9320115-a89d-4159-8416-6fcde98822b9",
    "isActive": true,
    "balance": "$2,489.22",
    "picture": "http://placehold.it/32x32",
    "age": 34,
    "eyeColor": "blue",
    "name": "Lana Finch",
    "gender": "female",
    "company": "PUSHCART",
    "email": "lanafinch@pushcart.com",
    "phone": "+1 (886) 407-2788",
    "address": "847 Albee Square, Whitestone, Rhode Island, 6120",
    "about": "Quis do non ut incididunt. Quis consectetur pariatur Lorem duis reprehenderit do. Occaecat in est dolore ipsum consequat in. Labore sint commodo officia culpa anim reprehenderit officia. Consequat duis nulla commodo voluptate deserunt eu sunt.\r\n",
    "registered": "2015-09-08T01:06:09 -02:00",
    "latitude": 52.540694,
    "longitude": -170.744891,
    "tags": [
      "proident",
      "esse",
      "reprehenderit",
      "Lorem",
      "mollit",
      "cupidatat",
      "cupidatat"
    ],
    "friends": [
      {
        "id": 0,
        "name": "Jarvis Oconnor"
      },
      {
        "id": 1,
        "name": "Enid Calderon"
      },
      {
        "id": 2,
        "name": "Valencia Delacruz"
      }
    ],
    "greeting": "Hello, Lana Finch! You have 6 unread messages.",
    "favoriteFruit": "strawberry"
  },
  {
    "_id": "59404dbca08548f51ec45e6b",
    "index": 1,
    "guid": "f2cc1a2b-919a-4952-8cbb-aa9f044001d9",
    "isActive": false,
    "balance": "$3,016.34",
    "picture": "http://placehold.it/32x32",
    "age": 22,
    "eyeColor": "green",
    "name": "Daniel Tate",
    "gender": "male",
    "company": "ZOXY",
    "email": "danieltate@zoxy.com",
    "phone": "+1 (813) 568-2706",
    "address": "365 Clermont Avenue, Benson, Minnesota, 8206",
    "about": "Tempor quis ex dolore commodo deserunt exercitation mollit deserunt cillum dolore velit consequat. Dolor ad mollit mollit sit cupidatat qui id laboris est. Dolore magna aliqua tempor ad et eu ullamco Lorem fugiat.\r\n",
    "registered": "2015-06-23T10:07:03 -02:00",
    "latitude": -68.11682,
    "longitude": -97.635221,
    "tags": [
      "quis",
      "ipsum",
      "ea",
      "esse",
      "amet",
      "commodo",
      "minim"
    ],
    "friends": [
      {
        "id": 0,
        "name": "Burnett Mitchell"
      },
      {
        "id": 1,
        "name": "Ruiz Ross"
      },
      {
        "id": 2,
        "name": "Bailey Alvarado"
      }
    ],
    "greeting": "Hello, Daniel Tate! You have 8 unread messages.",
    "favoriteFruit": "strawberry"
  },
  {
    "_id": "59404dbc73e3dad8e1c76d7e",
    "index": 2,
    "guid": "c57faa63-3bef-4f6f-8fdf-da1c2bdc0967",
    "isActive": false,
    "balance": "$1,600.42",
    "picture": "http://placehold.it/32x32",
    "age": 29,
    "eyeColor": "brown",
    "name": "Selma Jackson",
    "gender": "female",
    "company": "CODACT",
    "email": "selmajackson@codact.com",
    "phone": "+1 (922) 467-2841",
    "address": "964 Troutman Street, Nanafalia, Massachusetts, 5602",
    "about": "Dolor incididunt ullamco incididunt tempor anim adipisicing consequat do. Reprehenderit aliquip ullamco eu consectetur elit labore eiusmod elit culpa culpa culpa est ipsum. Pariatur labore incididunt in minim duis voluptate. Irure ullamco Lorem sunt deserunt. Irure eiusmod magna aute nulla cillum magna consectetur laboris consectetur occaecat. Esse irure non aliqua pariatur laborum eu Lorem et dolore nostrud. Nostrud Lorem pariatur incididunt ipsum consequat ut in.\r\n",
    "registered": "2016-02-27T03:47:41 -01:00",
    "latitude": 54.681896,
    "longitude": -5.911457,
    "tags": [
      "magna",
      "sunt",
      "nulla",
      "fugiat",
      "non",
      "cupidatat",
      "duis"
    ],
    "friends": [
      {
        "id": 0,
        "name": "Montoya Hill"
      },
      {
        "id": 1,
        "name": "Cross Byers"
      },
      {
        "id": 2,
        "name": "Valeria Berry"
      }
    ],
    "greeting": "Hello, Selma Jackson! You have 5 unread messages.",
    "favoriteFruit": "apple"
  },
  {
    "_id": "59404dbc109acdbcfb1886f7",
    "index": 3,
    "guid": "014eea30-051d-49c1-969e-e214e66612fa",
    "isActive": true,
    "balance": "$1,841.20",
    "picture": "http://placehold.it/32x32",
    "age": 27,
    "eyeColor": "brown",
    "name": "Gentry Mann",
    "gender": "male",
    "company": "GREEKER",
    "email": "gentrymann@greeker.com",
    "phone": "+1 (953) 443-2012",
    "address": "356 Verona Street, Diaperville, Michigan, 4708",
    "about": "Aliquip eiusmod in mollit velit magna irure ex mollit aliqua. Labore nulla incididunt esse laborum nulla ex. Est ullamco magna sit minim eu aute nulla. Do fugiat aute commodo non pariatur laborum ad nostrud nisi veniam minim nisi tempor laboris. Proident quis aute et pariatur exercitation aute ipsum proident qui cillum est irure. Tempor consectetur reprehenderit Lorem id fugiat ullamco id. Anim nulla sunt enim ipsum qui.\r\n",
    "registered": "2016-01-23T10:22:52 -01:00",
    "latitude": 45.886545,
    "longitude": 165.063216,
    "tags": [
      "non",
      "dolor",
      "esse",
      "ipsum",
      "labore",
      "anim",
      "magna"
    ],
    "friends": [
      {
        "id": 0,
        "name": "Peterson Nunez"
      },
      {
        "id": 1,
        "name": "Clayton Fletcher"
      },
      {
        "id": 2,
        "name": "Freeman Snow"
      }
    ],
    "greeting": "Hello, Gentry Mann! You have 9 unread messages.",
    "favoriteFruit": "apple"
  },
  {
    "_id": "59404dbc9b3ee21ee696f9e9",
    "index": 4,
    "guid": "6ad6cba9-ca33-4876-b7fb-6f024f06c6f5",
    "isActive": false,
    "balance": "$3,982.68",
    "picture": "http://placehold.it/32x32",
    "age": 38,
    "eyeColor": "blue",
    "name": "Leah Cannon",
    "gender": "female",
    "company": "SYNKGEN",
    "email": "leahcannon@synkgen.com",
    "phone": "+1 (947) 599-3879",
    "address": "198 Midwood Street, Tilleda, West Virginia, 7175",
    "about": "Aliquip eu velit aute do minim incididunt magna pariatur irure consectetur non. Nisi voluptate sunt ex reprehenderit ullamco aute consectetur esse ut excepteur minim excepteur. Aute adipisicing sint cupidatat occaecat sit do adipisicing nostrud fugiat. Incididunt laboris enim veniam commodo pariatur quis officia elit cupidatat ut. Sit ex duis exercitation excepteur labore duis esse ipsum officia ex labore. Velit est ex mollit commodo quis aute proident excepteur fugiat est esse.\r\n",
    "registered": "2014-06-02T05:55:41 -02:00",
    "latitude": 54.49165,
    "longitude": -144.499293,
    "tags": [
      "commodo",
      "elit",
      "officia",
      "deserunt",
      "Lorem",
      "ex",
      "proident"
    ],
    "friends": [
      {
        "id": 0,
        "name": "Valenzuela Barton"
      },
      {
        "id": 1,
        "name": "Mallory Stanley"
      },
      {
        "id": 2,
        "name": "Brooks Dale"
      }
    ],
    "greeting": "Hello, Leah Cannon! You have 3 unread messages.",
    "favoriteFruit": "strawberry"
  },
  {
    "_id": "59404dbca84a5e89eefeecb4",
    "index": 5,
    "guid": "ab6d8e9b-fde2-4aaf-b360-e47ace272d81",
    "isActive": true,
    "balance": "$1,356.61",
    "picture": "http://placehold.it/32x32",
    "age": 31,
    "eyeColor": "blue",
    "name": "Hopkins Cortez",
    "gender": "male",
    "company": "INTRAWEAR",
    "email": "hopkinscortez@intrawear.com",
    "phone": "+1 (820) 574-3945",
    "address": "576 Rogers Avenue, Guthrie, District Of Columbia, 3839",
    "about": "Do ex Lorem laboris esse in qui non et aute eu duis. Dolore ad fugiat pariatur laborum. Aliquip minim magna culpa occaecat anim incididunt ullamco adipisicing mollit mollit nulla et. Nulla do enim minim culpa reprehenderit in ut proident amet sint enim. Nisi sunt fugiat eu id cupidatat cillum irure anim commodo ex.\r\n",
    "registered": "2015-02-19T09:10:34 -01:00",
    "latitude": 66.939244,
    "longitude": 156.605874,
    "tags": [
      "ex",
      "velit",
      "non",
      "culpa",
      "cillum",
      "sint",
      "duis"
    ],
    "friends": [
      {
        "id": 0,
        "name": "Clara Luna"
      },
      {
        "id": 1,
        "name": "Oneal Compton"
      },
      {
        "id": 2,
        "name": "Espinoza Alvarez"
      }
    ],
    "greeting": "Hello, Hopkins Cortez! You have 8 unread messages.",
    "favoriteFruit": "banana"
  }
]
)gen_test_json";
