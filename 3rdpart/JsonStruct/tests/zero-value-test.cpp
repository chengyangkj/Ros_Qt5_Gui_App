/*
 * Copyright © 2020 Øystein Myrmo
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

#define CATCH_CONFIG_MAIN

#include "catch2/catch.hpp"

namespace
{

const char json_data[] = R"json(
{
  "f_pos_zero":  0,
  "f_neg_zero": -0,
  "d_pos_zero":  0,
  "d_neg_zero": -0
}
)json";

const char json_data_decimal[] = R"json(
{
  "f_pos_zero":  0.0,
  "f_neg_zero": -0.0,
  "d_pos_zero":  0.0,
  "d_neg_zero": -0.0
}
)json";

const char json_data_scientific[] = R"json(
{
  "f_pos_zero":  0e0,
  "f_neg_zero": -0e0,
  "d_pos_zero":  0e0,
  "d_neg_zero": -0e0
}
)json";

struct ZeroValueStruct
{
  float f_pos_zero;
  float f_neg_zero;
  double d_pos_zero;
  double d_neg_zero;

  JS_OBJECT(JS_MEMBER(f_pos_zero), JS_MEMBER(f_neg_zero), JS_MEMBER(d_pos_zero), JS_MEMBER(d_neg_zero));
};

// Notes:
// - This function is not tested on systems where double is 4 bytes.
// - The data is also cast to integer types for asserting since 0.0 normally is equal to -0.0.
void test_zero_value_parse(const char *const json)
{
  static float f_pos_zero = 0.0f;
  static float f_neg_zero = -0.0;
  static double d_pos_zero = 0.0;
  static double d_neg_zero = -0.0;

  ZeroValueStruct zero;
  JS::ParseContext pc(json);
  pc.parseTo(zero);
  REQUIRE(pc.error == JS::Error::NoError);

  REQUIRE(zero.f_pos_zero == f_pos_zero);
  REQUIRE(zero.f_neg_zero == f_neg_zero);
  REQUIRE(zero.d_pos_zero == d_pos_zero);
  REQUIRE(zero.d_neg_zero == d_neg_zero);

  REQUIRE(memcmp(&f_pos_zero, &zero.f_pos_zero, sizeof(float)) == 0);
  REQUIRE(memcmp(&f_neg_zero, &zero.f_neg_zero, sizeof(float)) == 0);
  REQUIRE(memcmp(&d_pos_zero, &zero.d_pos_zero, sizeof(double)) == 0);
  REQUIRE(memcmp(&d_neg_zero, &zero.d_neg_zero, sizeof(double)) == 0);

  // --------------------------
  // Keeping this code to debug on GCC later. foo2 == bar2 fails!
  // auto foo1 =
  // *reinterpret_cast<JS::Internal::ft::float_info<decltype(zero.f_pos_zero)>::uint_alias*>(&zero.f_pos_zero); auto foo2
  // = *reinterpret_cast<JS::Internal::ft::float_info<decltype(zero.f_neg_zero)>::uint_alias*>(&zero.f_neg_zero); auto
  // foo3 = *reinterpret_cast<JS::Internal::ft::float_info<decltype(zero.d_pos_zero)>::uint_alias*>(&zero.d_pos_zero);
  // auto foo4 =
  // *reinterpret_cast<JS::Internal::ft::float_info<decltype(zero.d_neg_zero)>::uint_alias*>(&zero.d_neg_zero);

  // auto bar1 = *reinterpret_cast<JS::Internal::ft::float_info<decltype(f_pos_zero)>::uint_alias*>(&f_pos_zero);
  // auto bar2 = *reinterpret_cast<JS::Internal::ft::float_info<decltype(f_neg_zero)>::uint_alias*>(&f_neg_zero);
  // auto bar3 = *reinterpret_cast<JS::Internal::ft::float_info<decltype(d_pos_zero)>::uint_alias*>(&d_pos_zero);
  // auto bar4 = *reinterpret_cast<JS::Internal::ft::float_info<decltype(d_neg_zero)>::uint_alias*>(&d_neg_zero);

  // REQUIRE(foo1 == bar1);
  // REQUIRE(foo2 == bar2);
  // REQUIRE(foo3 == bar3);
  // REQUIRE(foo4 == bar4);
  // --------------------------
}

TEST_CASE("test_zero_value", "[float conversion]")
{
  test_zero_value_parse(json_data);
  test_zero_value_parse(json_data_decimal);
  test_zero_value_parse(json_data_scientific);
}

} // namespace
