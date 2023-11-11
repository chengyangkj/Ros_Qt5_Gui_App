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
#include <tuple>

#include "catch2/catch.hpp"

#define SUB_ELEM(elem) SubElem<elem>(#elem)
#define SUB_LIST(...) std::make_tuple(__VA_ARGS__)

#define ELEM(elem) makeElem(#elem, &ELEMLIST_T::elem)
//#define ELEM(elem) Elem<ELEMLIST_T, elem, sizeof(#elem)>{#elem, &ELEMLIST_T::elem}

#define LIST(SUBLIST, ...)                                                                                             \
  template <typename ELEMLIST_T>                                                                                       \
  struct FOO                                                                                                           \
  {                                                                                                                    \
    static decltype(SUBLIST) &the_sublist()                                                                            \
    {                                                                                                                  \
      static auto list = SUBLIST;                                                                                      \
      return list;                                                                                                     \
    }                                                                                                                  \
    static decltype(std::make_tuple(__VA_ARGS__)) &getList()                                                           \
    {                                                                                                                  \
      static auto list = std::make_tuple(__VA_ARGS__);                                                                 \
      return list;                                                                                                     \
    }                                                                                                                  \
  }

namespace
{
template <typename T>
struct SubElem
{
  SubElem(const char *name)
    : name(name)
  {
  }
  const char *name;
  typedef T type;
};

template <typename T, typename M, size_t SIZE>
struct Elem
{
  const char *name;
  M T::*member;
  typedef T type;
};

template <typename T, typename M, size_t SIZE>
constexpr Elem<T, M, SIZE> makeElem(const char (&name)[SIZE], M T::*member)
{
  return Elem<T, M, SIZE>{name, member};
}

struct Cover
{
  int bar;
  int foo;
  // template<typename ELEMLIST_T>
  // struct FOO
  //{
  //  static decltype(std::make_tuple(SubElem<int>("int"))) &the_sublist()
  //  {
  //    static auto list = std::make_tuple(SubElem<int>("int")); return list;
  //  }
  //  static decltype(std::make_tuple(makeElem("bar", &ELEMLIST_T::bar), makeElem("foo", &ELEMLIST_T::foo)))
  //&getList()
  //  {
  //    static auto list = std::make_tuple(std::make_tuple(makeElem("bar", &ELEMLIST_T::bar), makeElem("foo",
  //&ELEMLIST_T::foo))); return list;
  //  }
  //};
  LIST(SUB_LIST(SUB_ELEM(int), SUB_ELEM(int), SUB_ELEM(int), SUB_ELEM(short)), ELEM(bar), ELEM(foo));
  FOO<Cover> f;
};
TEST_CASE("compiler_test", "")
{
  Cover c;
  JS_UNUSED(c);
  Cover::FOO<Cover>::getList();
  // Cover::FOO<Cover>::the_sublist();
  REQUIRE(true);
}
} // namespace
