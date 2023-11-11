include_directories(${JSON_STRUCT_INCLUDE_DIR})

include(CMakeRC.cmake)

cmrc_add_resource_library(
    external_json_resources
    ALIAS external_json::rc
    NAMESPACE external_json
    generated.json
    )

add_library(catch_main STATIC catch-main.cpp)
set(unit_test_sources
                           json-struct-test.cpp
                           json-struct-test-new.cpp
                           json-struct-external.cpp
                           json-struct-big-test.cpp
                           json-struct-verify.cpp
                           json-struct-serialize-tuple.cpp
                           json-struct-aliases-test.cpp
                           json-struct-serialize-test.cpp
                           json-struct-diff.cpp
                           json-struct-fail.cpp
                           json-struct-float.cpp
                           json-mias-mat.cpp
                           json-nullable-test.cpp
                           json-string-with-nullterminator-test.cpp
                           json-tokenizer-fail-test.cpp
                           json-tokenizer-partial-test.cpp
                           json-tokenizer-test.cpp
                           json-function-test.cpp
                           json-function-test-new.cpp
                           json-function-external-test.cpp
                           json-function-external-test-new.cpp
                           json-function-error-test.cpp
                           json-scope.cpp
                           members-size-test.cpp
                           compiler-test.cpp
                           meta-for-tokens.cpp
                           json-enum-test.cpp
                           json-meta-test.cpp
                           json-copy-test.cpp
                           multi-compilation-units-1.cpp
                           multi-compilation-units-2.cpp
                           multi-compilation-units-main.cpp
                           json-reformat.cpp
                           json-unordered-map.cpp
                           json-struct-utf8.cpp
                           json-struct-escape.cpp
                           json-struct-error-string.cpp
                           json-struct-polymorphic-map.cpp
                           json-struct-array-varlength.cpp
                           json-struct-stdint.cpp
                           json-struct-nested.cpp
                           json-struct-map-typehandler.cpp
                           json-tokenizer-invalid-json.cpp
                           )

add_executable(unit-tests ${unit_test_sources})
set_compiler_flags_for_target(unit-tests)
target_link_libraries(unit-tests PRIVATE catch_main external_json::rc)
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.16.0")
target_precompile_headers(unit-tests PRIVATE ../include/json_struct/json_struct.h catch2/catch.hpp)
endif()
add_test(NAME unit-tests COMMAND unit-tests)

# Tests for zero value floating points (both negative and positive zeros).
# The tests are run with different compiler options for floating point math.
add_executable(zero-value-test-fp-default zero-value-test.cpp)
set_compiler_flags_for_target(zero-value-test-fp-default)
add_executable(zero-value-test-fp-fast zero-value-test.cpp)
set_compiler_flags_for_target(zero-value-test-fp-fast)
add_test(NAME zero-value-test-fp-default COMMAND zero-value-test-fp-default)
add_test(NAME zero-value-test-fp-fast COMMAND zero-value-test-fp-fast)

if (MSVC)
  target_compile_options(zero-value-test-fp-fast PRIVATE /fp:fast)
else()
  target_compile_options(zero-value-test-fp-fast PRIVATE -ffast-math)
endif()

if ("${CMAKE_CXX_COMPILE_FEATURES}" MATCHES ".*cxx_std_17.*")
  add_executable(unit-tests-cxx17 json-optional.cpp ${unit_test_sources})
  if (NOT MSVC OR (MSVC_VERSION GREATER 1900))
    target_sources(unit-tests-cxx17 PRIVATE json-timepoint.cpp)
  endif()
  set_compiler_flags_for_target(unit-tests-cxx17)
  set_property(TARGET unit-tests-cxx17 PROPERTY CXX_STANDARD 17)
  target_compile_features(unit-tests-cxx17 PUBLIC cxx_std_17)
  target_link_libraries(unit-tests-cxx17 PRIVATE catch_main external_json::rc)
endif()

#add_executable(unit-tests-experimental json-struct-array-varlength.cpp)
#target_link_libraries(unit-tests-experimental PRIVATE catch_main)

add_custom_target(clangformat_tests
  COMMAND clang-format -i ${unit_test_sources}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
  SOURCES ${unit_test_sources}
  )
