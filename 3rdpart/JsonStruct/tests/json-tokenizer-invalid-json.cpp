#include <json_struct/json_struct.h>
#include "tokenizer-test-util.h"

#include "catch2/catch.hpp"

namespace 
{
static const char MESSAGE[] = R"html(<html>\r\n
	<head><title>400 Bad Request</title></head>\r\n
	<body>\r\n
	<center><h1>400 Bad Request</h1></center>\r\n
	<hr><center>cloudflare</center>\r\n
	</body>\r\n
	</html>\r\n)html";
TEST_CASE("html_to_tokenizer", "[tokenizer]")
{
  JS::ParseContext context(MESSAGE);
  JS::Map outMap;
  JS::Error error = context.parseTo(outMap);
  std::string error_str = context.makeErrorString();
  REQUIRE(error == JS::Error::EncounteredIllegalChar);
  REQUIRE(error_str.size());
}

}