/*
 * Copyright © 2016 Jorgen Lind
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <json_struct/json_struct.h>

#include "catch2/catch.hpp"

namespace
{

const char json_data1[] = R"json(
{
  "TheAlias": 55,
  "SomeOtherValue": 44
}
)json";

struct FirstAlias
{
  int ThePrimary = 0;
  int SomeOtherValue = 0;

  JS_OBJECT(JS_MEMBER_ALIASES(ThePrimary, "TheAlias"), JS_MEMBER(SomeOtherValue));
};

TEST_CASE("struct_aliases_checkPlain", "[json_struct][aliases]")
{
  JS::ParseContext context(json_data1);
  FirstAlias fa;
  auto error = context.parseTo(fa);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(fa.ThePrimary == 55);
  REQUIRE(fa.SomeOtherValue == 44);
}

struct ShadowAlias
{
  int TheAlias = 0;
  int SomeOtherValue = 0;

  JS_OBJECT(JS_MEMBER_ALIASES(TheAlias, "SomeOtherValue"), JS_MEMBER(SomeOtherValue));
};

TEST_CASE("struct_aliases_checkPlainShadow", "[json_struct][aliases]")
{
  JS::ParseContext context(json_data1);
  ShadowAlias sa;
  auto error = context.parseTo(sa);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(sa.TheAlias == 55);
  REQUIRE(sa.SomeOtherValue == 44);
}

const char json_data2[] = R"json(
{
  "SomeOtherValue": 44,
  "TheAlias": 55
}
)json";

struct TheSuper
{
  int TheAlias = 0;
  JS_OBJECT(JS_MEMBER(TheAlias));
};

struct TheSub : public TheSuper
{
  int SomeOtherValue = 0;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(TheSuper)), JS_MEMBER_ALIASES(SomeOtherValue, "TheAlias"));
};

TEST_CASE("struct_aliases_checkSuperShadow", "[json_struct][aliases]")
{
  JS::ParseContext context(json_data1);
  TheSub sa;
  auto error = context.parseTo(sa);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(sa.TheAlias == 55);
  REQUIRE(sa.SomeOtherValue == 44);

  context = JS::ParseContext(json_data2);
  sa = TheSub();
  error = context.parseTo(sa);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(sa.TheAlias == 55);
  REQUIRE(sa.SomeOtherValue == 44);
}

const char recursive_alias[] = R"json(
{
  "first": "first",
  "second": {
    "one": "one",
    "two": "two",
    "three": "three"
  },
  "third": "third"
}
)json";

struct Second
{
  std::string one;
  std::string two_primary;
  std::string three;

  JS_OBJECT(JS_MEMBER(one), JS_MEMBER_ALIASES(two_primary, "two"), JS_MEMBER(three));
};
struct First
{
  std::string first;
  Second second;
  std::string third;
  JS_OBJECT(JS_MEMBER(first), JS_MEMBER(second), JS_MEMBER(third));
};

TEST_CASE("struct_aliases_checkRecursiveShadow", "[json_struct][aliases]")
{
  JS::ParseContext context(recursive_alias);
  First f;
  auto error = context.parseTo(f);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(f.first == "first");
  REQUIRE(f.second.two_primary == "two");
  REQUIRE(f.second.three == "three");
  REQUIRE(f.third == "third");
}

} // namespace
