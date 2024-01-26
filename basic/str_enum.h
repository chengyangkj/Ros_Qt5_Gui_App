#pragma once
#include <stdio.h>
#include <string.h>
#include <string>
// expansion macro for enum value definition
#define ENUM_VALUE(name, assign) name assign,

// expansion macro for enum to string conversion
#define ENUM_CASE(name, assign) \
  case name:                    \
    return #name;

// expansion macro for string to enum conversion
#define ENUM_STRCMP(name, assign) \
  if (!strcmp(str, #name))        \
    return name;

/// declare the access function and define enum values
#define DECLARE_ENUM(EnumType, ENUM_DEF)                  \
  enum EnumType { ENUM_DEF(ENUM_VALUE) };                 \
  inline const std::string ToString(enum EnumType dummy); \
  inline enum EnumType Get##EnumType##Value(const char *str);

/// define the access function names
#define DEFINE_ENUM(EnumType, ENUM_DEF)                 \
  const std::string ToString(enum EnumType value) {     \
    switch (value) {                                    \
      ENUM_DEF(ENUM_CASE)                               \
      default:                                          \
        return ""; /* handle input error */             \
    }                                                   \
  }                                                     \
  enum EnumType Get##EnumType##Value(const char *str) { \
    ENUM_DEF(ENUM_STRCMP)                               \
    return (enum EnumType)0; /* handle input error */   \
  }
