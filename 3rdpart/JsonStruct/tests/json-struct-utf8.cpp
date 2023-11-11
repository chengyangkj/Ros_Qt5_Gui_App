/*
 * Copyright � 2021 Jorgen Lind
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

static const char json_data1[] = R"json({
  "One": "j\u00f8rgen",
  "Two": "j\u00f8rgen\u00f8",
  "Three": "j\u00f8rgen\u012",
  "Four": "j\u00f8rgen\u01",
  "Five": "j\u00f8rgen\u0"
})json";

struct Struct
{
  std::string One;
  std::string Two;
  std::string Three;
  std::string Four;
  std::string Five;
  JS_OBJ(One, Two, Three, Four, Five);
};

TEST_CASE("test_simple_utf8", "[json_struct][utf-8]")
{
  JS::ParseContext context(json_data1);
  Struct substruct;
  auto error = context.parseTo(substruct);

  REQUIRE(error == JS::Error::NoError);
  REQUIRE(substruct.One == u8"j\u00f8rgen");
  REQUIRE(substruct.Two == u8"j\u00f8rgen\u00f8");
  REQUIRE(substruct.Three == u8"j\u00f8rgen\\u012");
  REQUIRE(substruct.Four == u8"j\u00f8rgen\\u01");
  REQUIRE(substruct.Five == u8"j\u00f8rgen\\u0");
}
} // namespace
