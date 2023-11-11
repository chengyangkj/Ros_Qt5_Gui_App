/*
 * Copyright © 2017 Jorgen Lind
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

namespace json_struct_verify
{

static const char json_data1[] = "{\n"
                                 "\"StringNode\": \"Some test data\",\n"
                                 "\"NumberNode\": 4676\n"
                                 "}\n";

struct ContainsStringNode
{
  std::string StringNode;

  JS_OBJECT(JS_MEMBER(StringNode));
};

struct SubStructVerify : public ContainsStringNode
{
  int NumberNode;

  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(ContainsStringNode)), JS_MEMBER(NumberNode));
};

TEST_CASE("testSimpleOneMember", "[json_struct][json_struct_verify]")
{
  JS::ParseContext context(json_data1);
  SubStructVerify substruct;
  auto error = context.parseTo(substruct);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(substruct.StringNode == "Some test data");
  REQUIRE(substruct.NumberNode == 4676);
}

static const char json_data2[] = "{\n"
                                 "\"ThisWillBeUnassigned\": \"Some data\",\n"
                                 "\"StringNode\": \"Some test data\"\n"
                                 "}\n";

TEST_CASE("testSimpleVerifyMissingMemberInStruct", "[json_struct][json_struct_verify]")
{
  JS::ParseContext context(json_data2);
  ContainsStringNode containsString;
  auto error = context.parseTo(containsString);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(containsString.StringNode == "Some test data");
  REQUIRE(context.missing_members.size() == 1);
  REQUIRE(context.missing_members.front() == "ThisWillBeUnassigned");
}

static const char json_data3[] = "{\n"
                                 "\"Field1\": 1\n,"
                                 "\"Field3\": 3\n"
                                 "}\n";

struct RequiredMemberStruct
{
  int Field1;
  int Field2;
  int Field3;

  JS_OBJECT(JS_MEMBER(Field1), JS_MEMBER(Field2), JS_MEMBER(Field3));
};

TEST_CASE("testSimpleVerifyMissingRequiredMemberInStruct", "[json_struct][json_strut_verify]")
{
  JS::ParseContext context(json_data3);
  RequiredMemberStruct requiredMemberStruct;
  auto error = context.parseTo(requiredMemberStruct);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(requiredMemberStruct.Field3 == 3);
  REQUIRE(context.unassigned_required_members.size() == 1);
  REQUIRE(context.unassigned_required_members.front() == "Field2");
}

static const char json_data4[] = "{\n"
                                 "\"StringNode\": \"Some test data\",\n"
                                 "\"NumberNode\": 342,\n"
                                 "\"SubNode\": \"This should be in subclass\"\n"
                                 "}\n";

struct SuperClass
{
  std::string StringNode;
  int NumberNode;
  JS_OBJECT(JS_MEMBER(StringNode), JS_MEMBER(NumberNode));
};

struct SubClass : public SuperClass
{
  std::string SubNode;
  int SubNode2;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(SuperClass)), JS_MEMBER(SubNode), JS_MEMBER(SubNode2));
};

TEST_CASE("testClassHirarchyVerifyMissingMemberInStruct", "[json_struct][json_strut_verify]")
{
  JS::ParseContext context(json_data4);
  SubClass subClass;
  auto error = context.parseTo(subClass);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(subClass.NumberNode == 342);
  REQUIRE(subClass.SubNode == "This should be in subclass");
  REQUIRE(context.unassigned_required_members.size() == 1);
  REQUIRE(context.unassigned_required_members.front() == "SubNode2");
}

struct SuperSuperClass
{
  int SuperSuper;
  JS_OBJECT(JS_MEMBER(SuperSuper));
};

struct SuperClass2 : public SuperSuperClass
{
  std::string Super;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(SuperSuperClass)), JS_MEMBER(Super));
};

struct RegularClass : public SuperClass2
{
  int Regular;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(SuperClass2)), JS_MEMBER(Regular));
};

static const char json_data5[] = "{\n"
                                 "\"SuperSuper\": 5,\n"
                                 "\"Regular\": 42\n"
                                 "}\n";

TEST_CASE("testClassHIrarchyVerifyMissingDataForStruct", "[json_struct][json_strut_verify]")
{
  JS::ParseContext context(json_data5);
  RegularClass regular;
  auto error = context.parseTo(regular);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(context.unassigned_required_members.size() == 1);
  REQUIRE(context.unassigned_required_members.front() == "SuperClass2::Super");
}

static const char json_data6[] = "{\n"
                                 "\"SuperSuper\": 5,\n"
                                 "\"Super\": \"This is super\",\n"
                                 "\"SuperSuperSuper\": 42,\n"
                                 "\"Regular\": 42\n"
                                 "}\n";

TEST_CASE("testClassHirarchyVerifyMissingMemberInStruct2", "[json_struct][json_strut_verify]")
{
  JS::ParseContext context(json_data6);
  RegularClass regular;
  auto error = context.parseTo(regular);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(context.missing_members.size() == 1);
  REQUIRE(context.missing_members.front() == "SuperSuperSuper");
}

struct A
{
  int a;
  JS_OBJECT(JS_MEMBER(a));
};

struct B : public A
{
  float b;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(A)), JS_MEMBER(b));
};

struct D
{
  int d;
  JS_OBJECT(JS_MEMBER(d));
};

struct E : public D
{
  double e;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(D)), JS_MEMBER(e));
};

struct F : public E
{
  int f;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(E)), JS_MEMBER(f));
};
struct G
{
  std::string g;
  JS_OBJECT(JS_MEMBER(g));
};

struct Subclass : public B, public F, public G
{
  int h;
  JS_OBJECT_WITH_SUPER(JS_SUPER_CLASSES(JS_SUPER_CLASS(B), JS_SUPER_CLASS(F), JS_SUPER_CLASS(G)), JS_MEMBER(h));
};

static const char json_data7[] = "{\n"
                                 "\"a\": 4,\n"
                                 "\"b\": 5.5,\n"
                                 "\"d\": 127,\n"
                                 "\"f\": 345,\n"
                                 "\"g\": \"a\",\n"
                                 "\"h\": 987\n"
                                 "}\n";

TEST_CASE("testClassHirarchyVerifyMissingDataForStructDeep", "[json_struct][json_strut_verify]")
{
  JS::ParseContext context(json_data7);
  Subclass subclass;
  auto error = context.parseTo(subclass);
  REQUIRE(error == JS::Error::NoError);

  REQUIRE(context.unassigned_required_members.size() == 1);
  REQUIRE(context.unassigned_required_members.front() == "E::e");
}

} // namespace json_struct_verify
