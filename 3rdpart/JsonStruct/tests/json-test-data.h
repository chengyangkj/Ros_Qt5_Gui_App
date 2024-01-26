/*
 * Copyright © 2012 Jorgen Lind
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided \"as
 * is\" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#ifndef JSON_TEST_DATA_H
#define JSON_TEST_DATA_H

#include "assert.h"

const char json_data1[] =
    "{\n"
    "   \"foo\": \"bar\",\n"
    "   \"color\": \"red\"\n"
    "   weather: \"clear\"\n"
    "   weather1 : \"clear1\"\n"
    "   ToBeTrue: true,\n"
    "   HeresANull : null\n"
    "   ThisIsFalse: false,\n"
    "\n"
    "   EscapedString: \"contains \\\"\",\n"
    "  \"\\\"EscapedName\\\"\": true,\n"
    "  \"EscapedProp\": \"\\\"Hello\\\"\",\n"
    "   ThisIsANumber: 3.14\n"
    "   ThisIsAnObject: {\n"
    "     ThisIsASubType: \"red\"\n"
    "   },\n"
    "   AnotherProp: \"prop\"\n"
    "   ThisIsAnotherObject: {\n"
    "     ThisIsAnotherASubType: \"blue\"\n"
    "   },\n"
    "   ThisIsAnArray: [\n"
    "     12.4,\n"
    "     3,\n"
    "     43.2\n"
    "   ]\n"
    "   ThisIsAnObjectArray: [\n"
    "     { Test1: \"Test2\", Test3: \"Test4\" },\n"
    "     { Test5: true, Test7: false }\n"
    "   ]\n"
    "}\n";

const char json_data2[] =
    "{\n"
    "  \"StringNode\": \"Some test data\",\n"
    "  \"NumberNode\": 4676.4,\n"
    "  \"NullNode\": null,\n"
    "  \"BooleanTrue\": true,\n"
    "  \"BooleanFalse\": false,\n"
    "  \"Object\": {\n"
    "    \"SomeSubObjectProp\": \"RED\"\n"
    "  },\n"
    "  \"Array\": [\n"
    "    \"String\",\n"
    "    null,\n"
    "    true,\n"
    "    {\n"
    "      \"SomeOtherObjectProp\": \"GREEN\"\n"
    "    }\n"
    "  ],\n"
    "  \"LastStringNode\": \"More test data\"\n"
    "}\n";

#endif  //JSON_TEST_DATA_H
