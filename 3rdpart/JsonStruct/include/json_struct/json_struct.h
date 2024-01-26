/*
* Copyright © 2020 Jørgen Lind

* Permission to use, copy, modify, distribute, and sell this software and its
* documentation for any purpose is hereby granted without fee, provided that
* the above copyright notice appear in all copies and that both that copyright
* notice and this permission notice appear in supporting documentation, and
* that the name of the copyright holders not be used in advertising or
* publicity pertaining to distribution of the software without specific,
* written prior permission.  The copyright holders make no representations
* about the suitability of this software for any purpose.  It is provided "as
* is" without express or implied warranty.

* THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
* INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
* EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
* CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
* DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
* TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
* OF THIS SOFTWARE.
*/

/*! \file */

/*! \mainpage json_struct
 *
 * json_struct is a set of classes meant for simple and efficient parse,
 * tokenize and validate json.
 *
 * json_struct support parsing json into a stream of tokens using the \ref
 * tokenizer "JS::Tokenizer" api, or parsing json into c++ structures using the
 * \ref js_struct "JS_OBJECT" api.
 */

/*! \page tokenizer Parsing json using JS::Tokenizer
 *
 * Tokenizing json JS::Tokenizer can be used to extract tokens
 * from a json stream.  Tokens does not describe a full object, but only
 * key-value pairs. So a object would be: "some key" and object start. Then the
 * next token would be the first key value pair of that object. Then at some
 * point the object is finished, and an object end value with no key would be
 * the token.
 *
 * Arrays would be expressed in a similar fashion, but the tokens would have no
 * key, and each element in the array would be tokens with only a value
 * specified.
 *
 * The root object would be a token with no key data, but only object or array
 * start
 *
 * A crude example of this is viewed in \ref simple_tokenize.cpp here:
 * \include simple_tokenize.cpp
 *
 * Tokenizing json in this way allows you parse arbitrary large json data.
 * Also the tokenizer has mechanisms for asking for more data, making it easy
 * to stream json data. Using this interface to parse json is a bit verbose and
 * requires the application code to keep some extra state. json_struct also has
 * functionality for parsing json data directly into c++ structures. This is
 * done by adding some metadata to the structure, or by adding a template
 * specialisation of a class.  \ref js_struct "describes this" in more detail.
 */

/*! \example simple_tokenize.cpp
 *
 * This example show very basic usage of how JS::Tokenizer can be used
 */

/*! \example simple_struct.cpp
 *
 * This example shows basic usage of parsing Json directly into structs
 */

/*! \page js_struct Parsing json into C++ structs
 *
 * json_struct makes it very easy to put your json data into c++ structures or
 * take data from c++ structures and generate json.
 *
 * This is best shown with an example: \include simple_struct.cpp
 *
 * The two interesting sections here are the lines are the:
 * \code{.cpp}
 *    JS_OBJECT(JS_MEMBER(key),
 *              JS_MEMBER(number),
 *              JS_MEMBER(boolean));
 * \endcode
 *
 * and
 *
 * \code{.cpp}
 *    JS::ParseContext parseContext(json);
 *    JsonData dataStruct;
 *    parseContext.parseTo(dataStruct);
 * \endcode
 *
 * The JS_OBJECT call inside the JsonData struct will create a nested struct
 * declaration inside the JsonData struct. This nested struct will expose some
 * meta data about the struct, exposing the names of the members at runtime.
 * json_struct can then use this runtime information to populate the struct.
 *
 * Populating the struct is done by first creating a JS::ParseContext. The
 * JS::ParseContext contains a JS::Tokenizer. This tokenizer is what the actual
 * state holder for the parsing. If allowing using '\n' instead of ',' to
 * seperate object and array elements, then this should be set on the
 * JS::Tokenizer.
 *
 * Since types will dictate the schema of the input json, the JS::ParseContext
 * will expose a list containing what members where not populated by the input
 * json, and what member in the input json that did not have any member to
 * populate.
 */

#ifndef JSON_STRUCT_H
#define JSON_STRUCT_H

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#ifdef _MSC_VER
  #include <intrin.h>
#endif

#if __cplusplus > 199711L || (defined(_MSC_VER) && _MSC_VER > 1800)
  #define JS_STD_UNORDERED_MAP 1
#endif
#ifdef JS_STD_UNORDERED_MAP
  #include <unordered_map>
#endif

#ifndef JS_STD_OPTIONAL
  #if defined(__APPLE__)
    #if __clang_major__ > 9 && __cplusplus >= 201703L
      #define JS_STD_OPTIONAL 1
    #endif
  #elif defined(_MSC_VER) && _MSC_VER >= 1910 && _HAS_CXX17
    #define JS_STD_OPTIONAL 1
  #elif __cplusplus >= 201703L
    #define JS_STD_OPTIONAL 1
  #endif
#endif

#ifdef JS_STD_OPTIONAL
  #include <optional>
#endif

#ifdef JS_STD_TIMEPOINT
  #include <chrono>
  #include <type_traits>
#endif

#ifndef JS_IF_CONSTEXPR
  #if __cpp_if_constexpr
    #define JS_IF_CONSTEXPR(exp) if constexpr (exp)
  #elif defined(_MSC_VER)
    #define JS_IF_CONSTEXPR(exp) __pragma(warning(push)) __pragma(warning(disable : 4127)) if (exp) __pragma(warning(pop))
  #else
    #define JS_IF_CONSTEXPR(exp) if (exp)
  #endif
#endif

#if JS_NO_NODISCARD
  #define JS_NODISCARD
#else
  #if __cplusplus >= 201703L
    #define JS_NODISCARD [[nodiscard]]
  #else
    #define JS_NODISCARD
  #endif
#endif

#if defined(min) || defined(max)
  #error min or max macro is defined. Make sure these are not defined before including json_struct.h.\
 Use "#define NOMINMAX 1" before including Windows.h
#endif

#define JS_UNUSED(x) (void)(x)

#ifndef JS
  #define JS JS
#endif

namespace JS {
/*!
 *  \brief Pointer to data
 *
 *  DataRef is used to refere to some data inside a json string. It holds the
 *  start posisition of the data, and its size.
 */
struct DataRef {
  /*!
   * Constructs a null Dataref pointing to "" with size 0.
   */
  constexpr explicit DataRef()
      : data(""), size(0) {
  }

  /*!
   * Constructs a DataRef pointing to data and size.
   * \param data points to start of data.
   * \param size size of data.
   */
  constexpr explicit DataRef(const char *data, size_t size)
      : data(data), size(size) {
  }

  /*!  Cobstructs a DataRef pointing to an array. This will \b NOT look for
   * the null terminator, but just initialize the DataRef to the size of the
   * array - 1. This function is intended to be used with string literals.
   * \param data  start of the data.
   */
  template <size_t N>
  constexpr explicit DataRef(const char (&data)[N])
      : data(data), size(N - 1) {
  }

  explicit DataRef(const std::string &str)
      : data(&str[0]), size(str.size()) {
  }

  explicit DataRef(const char *data)
      : data(data), size(strlen(data)) {
  }

  const char *data;
  size_t size;
};

enum class Type : unsigned char {
  Error,
  String,
  Ascii,
  Number,
  ObjectStart,
  ObjectEnd,
  ArrayStart,
  ArrayEnd,
  Bool,
  Null,
  Verbatim
};

struct Token {
  Token();

  DataRef name;
  DataRef value;
  Type name_type;
  Type value_type;
};

namespace Internal {
struct IntermediateToken {
  IntermediateToken()
      : active(false), name_type_set(false), data_type_set(false) {
  }

  void clear() {
    if (!active)
      return;
    active = false;
    name_type_set = false;
    data_type_set = false;
    name_type = Type::Error;
    data_type = Type::Error;
    name.clear();
    data.clear();
  }

  bool active : 1;
  bool name_type_set : 1;
  bool data_type_set : 1;
  Type name_type = Type::Error;
  Type data_type = Type::Error;
  std::string name;
  std::string data;
};
enum Lookup {
  StrEndOrBackSlash = 1,
  AsciiLetters = 2,
  WhiteSpaceOrNull = 4,
  PlusOrMinus = 8,
  Digits = 16,
  HatUnderscoreAprostoph = 32,
  NumberEnd = 64
};

static inline const unsigned char *lookup() {
  static const unsigned char tmp[] = {
      /*0*/ 4, 0, 0, 0, 0, 0, 0, 0,
      /*8*/ 0, 4, 4, 0, 0, 4, 0, 0,
      /*16*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*24*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*32*/ 4, 0, 1, 0, 0, 0, 0, 0,
      /*40*/ 0, 0, 0, 8 | 64, 0, 8 | 64, 64, 0,
      /*48*/ 16 | 64, 16 | 64, 16 | 64, 16 | 64, 16 | 64, 16 | 64, 16 | 64, 16 | 64,
      /*56*/ 16 | 64, 16 | 64, 0, 0, 0, 0, 0, 0,
      /*64*/ 0, 2, 2, 2, 2, 2 | 64, 2, 2,
      /*72*/ 2, 2, 2, 2, 2, 2, 2, 2,
      /*80*/ 2, 2, 2, 2, 2, 2, 2, 2,
      /*88*/ 2, 2, 2, 0, 1, 0, 32, 32,
      /*96*/ 32, 2, 2, 2, 2, 2 | 64, 2, 2,
      /*104*/ 2, 2, 2, 2, 2, 2, 2, 2,
      /*112*/ 2, 2, 2, 2, 2, 2, 2, 2,
      /*120*/ 2, 2, 2, 0, 0, 0, 0, 0,
      /*128*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*136*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*144*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*152*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*160*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*168*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*176*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*184*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*192*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*200*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*208*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*216*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*224*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*232*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*240*/ 0, 0, 0, 0, 0, 0, 0, 0,
      /*248*/ 0, 0, 0, 0, 0, 0, 0, 0};
  return tmp;
}
}  // namespace Internal

enum class Error : unsigned char {
  NoError,
  NeedMoreData,
  InvalidToken,
  ExpectedPropertyName,
  ExpectedDelimiter,
  ExpectedDataToken,
  ExpectedObjectStart,
  ExpectedObjectEnd,
  ExpectedArrayStart,
  ExpectedArrayEnd,
  IllegalPropertyName,
  IllegalPropertyType,
  IllegalDataValue,
  EncounteredIllegalChar,
  NodeNotFound,
  MissingPropertyMember,
  MissingFunction,
  FailedToParseBoolean,
  FailedToParseDouble,
  FailedToParseFloat,
  FailedToParseInt,
  UnassignedRequiredMember,
  NonContigiousMemory,
  ScopeHasEnded,
  KeyNotFound,
  DuplicateInSet,
  UnknownError,
  UserDefinedErrors
};

namespace Internal {
class ErrorContext {
 public:
  size_t line = 0;
  size_t character = 0;
  Error error = Error::NoError;
  std::string custom_message;
  std::vector<std::string> lines;

  void clear() {
    line = 0;
    character = 0;
    error = Error::NoError;
    lines.clear();
  }
};

template <typename T>
struct CallbackContainer;
}  // namespace Internal

template <typename T>
class RefCounter {
 public:
  RefCounter()
      : callbackContainer(nullptr), index(0) {
  }

  RefCounter(size_t index, Internal::CallbackContainer<T> *callbackContainer)
      : callbackContainer(callbackContainer), index(index) {
    inc();
  }
  RefCounter(const RefCounter<T> &other)
      : callbackContainer(other.callbackContainer) {
    inc();
  }

  RefCounter<T> &operator=(const RefCounter<T> &other) {
    dec();
    callbackContainer = other.callbackContainer;
    index = other.index;
    inc();
    return *this;
  }

  ~RefCounter() {
    dec();
  }

 private:
  void inc();
  void dec();
  Internal::CallbackContainer<T> *callbackContainer;
  size_t index;
};

template <typename T>
class Callback {
 public:
  Callback()
      : ref(0) {
  }

  Callback(std::function<T> &callback)
      : ref(0), callback(callback) {
  }
  Callback(const Callback<T> &other)
      : ref(other.ref.load()), callback(other.callback) {
  }
  Callback &operator=(const Callback<T> &other) {
    ref.store(other.ref.load());
    callback = other.callback;
    return *this;
  }

  void inc() {
    ++ref;
  }
  void dec() {
    --ref;
  }

  std::atomic<int> ref;
  std::function<T> callback;
};

namespace Internal {
template <typename T>
struct CallbackContainer {
 public:
  const RefCounter<T> addCallback(std::function<T> &callback) {
    for (size_t i = 0; i < vec.size(); i++) {
      if (vec[i].ref.load() == 0) {
        vec[i].callback = callback;
        return RefCounter<T>(i, this);
      }
    }
    vec.push_back(Callback<T>(callback));
    return RefCounter<T>(vec.size() - 1, this);
  }

  template <typename... Ts>
  void invokeCallbacks(Ts &... args) {
    for (auto &callbackHandler : vec) {
      if (callbackHandler.ref.load()) {
        callbackHandler.callback(args...);
      }
    }
  }
  void inc(size_t index) {
    assert(index < vec.size());
    ++vec[index].ref;
  }
  void dec(size_t index) {
    assert(index < vec.size());
    assert(vec[index].ref.load() != 0);
    --vec[index].ref;
  }

 private:
  std::vector<Callback<T>> vec;
};

struct ScopeCounter {
  JS::Type type;
  uint16_t depth;
  inline void handleType(JS::Type in_type) {
    if (type == JS::Type::ArrayStart || type == JS::Type::ObjectStart) {
      if (in_type == type)
        depth++;
      else if (in_type == JS::Type(static_cast<int>(type) + 1))
        depth--;
    } else {
      depth--;
    }
  }
};
}  // namespace Internal

template <typename T>
inline void RefCounter<T>::inc() {
  if (callbackContainer)
    callbackContainer->inc(index);
}

template <typename T>
inline void RefCounter<T>::dec() {
  if (callbackContainer)
    callbackContainer->dec(index);
}

class Tokenizer;
typedef RefCounter<void(const char *)> ReleaseCBRef;
typedef RefCounter<void(Tokenizer &)> NeedMoreDataCBRef;

class Tokenizer {
 public:
  Tokenizer();

  void allowAsciiType(bool allow);
  void allowNewLineAsTokenDelimiter(bool allow);
  void allowSuperfluousComma(bool allow);

  void addData(const char *data, size_t size);
  template <size_t N>
  void addData(const char (&data)[N]);
  void addData(const std::vector<Token> *parsedData);
  void resetData(const char *data, size_t size, size_t index);
  void resetData(const std::vector<Token> *parsedData, size_t index);
  size_t registeredBuffers() const;

  NeedMoreDataCBRef registerNeedMoreDataCallback(std::function<void(Tokenizer &)> callback);
  ReleaseCBRef registerReleaseCallback(std::function<void(const char *)> &callback);
  Error nextToken(Token &next_token);
  const char *currentPosition() const;

  void copyFromValue(const Token &token, std::string &to_buffer);
  void copyIncludingValue(const Token &token, std::string &to_buffer);

  void pushScope(JS::Type type);
  void popScope();
  JS::Error goToEndOfScope(JS::Token &token);

  std::string makeErrorString() const;
  void setErrorContextConfig(size_t lineContext, size_t rangeContext);
  Error updateErrorContext(Error error, const std::string &custom_message = std::string());
  const Internal::ErrorContext &errorContext() const {
    return error_context;
  }

 private:
  enum class InTokenState : unsigned char {
    FindingName,
    FindingDelimiter,
    FindingData,
    FindingTokenEnd
  };

  enum class InPropertyState : unsigned char {
    NoStartFound,
    FindingEnd,
    FoundEnd
  };

  void resetForNewToken();
  void resetForNewValue();
  Error findStringEnd(const DataRef &json_data, size_t *chars_ahead);
  Error findAsciiEnd(const DataRef &json_data, size_t *chars_ahead);
  Error findNumberEnd(const DataRef &json_data, size_t *chars_ahead);
  Error findStartOfNextValue(Type *type, const DataRef &json_data, size_t *chars_ahead);
  Error findDelimiter(const DataRef &json_data, size_t *chars_ahead);
  Error findTokenEnd(const DataRef &json_data, size_t *chars_ahead);
  void requestMoreData();
  void releaseFirstDataRef();
  Error populateFromDataRef(DataRef &data, Type &type, const DataRef &json_data);
  static void populate_annonymous_token(const DataRef &data, Type type, Token &token);
  Error populateNextTokenFromDataRef(Token &next_token, const DataRef &json_data);

  InTokenState token_state = InTokenState::FindingName;
  InPropertyState property_state = InPropertyState::NoStartFound;
  Type property_type = Type::Error;
  bool is_escaped : 1;
  bool allow_ascii_properties : 1;
  bool allow_new_lines : 1;
  bool allow_superfluous_comma : 1;
  bool expecting_prop_or_annonymous_data : 1;
  bool continue_after_need_more_data : 1;
  size_t cursor_index;
  size_t current_data_start;
  size_t line_context;
  size_t line_range_context;
  size_t range_context;
  Internal::IntermediateToken intermediate_token;
  std::vector<DataRef> data_list;
  std::vector<Internal::ScopeCounter> scope_counter;
  std::vector<Type> container_stack;
  Internal::CallbackContainer<void(const char *)> release_callbacks;
  Internal::CallbackContainer<void(Tokenizer &)> need_more_data_callbacks;
  std::vector<std::pair<size_t, std::string *>> copy_buffers;
  const std::vector<Token> *parsed_data_vector;
  Internal::ErrorContext error_context;
};

namespace Internal {
template <size_t SIZE>
struct StringLiteral {
  const char *data;
  enum size_enum {
    size = SIZE
  };
};
template <size_t SIZE>
constexpr StringLiteral<SIZE - 1> makeStringLiteral(const char (&literal)[SIZE]) {
  return {literal};
}
}  // namespace Internal

class SerializerOptions {
 public:
  enum Style : unsigned char {
    Pretty,
    Compact
  };

  SerializerOptions(Style style = Style::Pretty);

  int shiftSize() const;
  void setShiftSize(unsigned char set);

  Style style() const;
  void setStyle(Style style);

  bool convertAsciiToString() const;
  void setConvertAsciiToString(bool set);

  unsigned char depth() const;
  void setDepth(int depth);

  void skipDelimiter(bool skip);

  const std::string &prefix() const;
  const std::string &tokenDelimiter() const;
  const std::string &valueDelimiter() const;
  const std::string &postfix() const;

 private:
  uint8_t m_shift_size;
  uint8_t m_depth;
  Style m_style;
  bool m_convert_ascii_to_string;

  std::string m_prefix;
  std::string m_token_delimiter;
  std::string m_value_delimiter;
  std::string m_postfix;
};

#if __cplusplus >= 201403L
template <SerializerOptions::Style S = SerializerOptions::Style::Pretty>
const static SerializerOptions DEFAULT_OPS =
    []() {  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables,cert-err58-cpp)
      SerializerOptions ops(S);
      return ops;
    }();
#endif

class SerializerBuffer {
 public:
  SerializerBuffer()
      : buffer(nullptr), size(0), used(0) {}
  SerializerBuffer(char *buffer, size_t size)
      : buffer(buffer), size(size), used(0) {}
  size_t free() const {
    return size - used;
  }
  void append(const char *data, size_t size);
  template <size_t SIZE>
  void append(const char *data);
  char *buffer;
  size_t size;
  size_t used;
};

class Serializer;
typedef RefCounter<void(Serializer &)> BufferRequestCBRef;
class Serializer {
 public:
  Serializer();
  Serializer(char *buffer, size_t size);

  void setBuffer(char *buffer, size_t size);
  void setOptions(const SerializerOptions &option);
  SerializerOptions options() const {
    return m_option;
  }

  bool write(const Token &token);
  bool write(const char *data, size_t size);
  bool write(const std::string &str) {
    return write(str.c_str(), str.size());
  }
  template <size_t SIZE>
  inline bool write(const Internal::StringLiteral<SIZE> &strLiteral);

  const BufferRequestCBRef addRequestBufferCallback(std::function<void(Serializer &)> callback);
  const SerializerBuffer &currentBuffer() const;

 private:
  void askForMoreBuffers();
  void markCurrentSerializerBufferFull();
  bool writeAsString(const DataRef &data);
  bool write(Type type, const DataRef &data);

  Internal::CallbackContainer<void(Serializer &)> m_request_buffer_callbacks;
  SerializerBuffer m_current_buffer;

  bool m_first;
  bool m_token_start;
  SerializerOptions m_option;
};

// IMPLEMENTATION

inline Token::Token()
    : name(), value(), name_type(Type::String), value_type(Type::String) {
}

inline Tokenizer::Tokenizer()
    : is_escaped(false), allow_ascii_properties(false), allow_new_lines(false), allow_superfluous_comma(false), expecting_prop_or_annonymous_data(false), continue_after_need_more_data(false), cursor_index(0), current_data_start(0), line_context(4), line_range_context(256), range_context(38), parsed_data_vector(nullptr) {
  container_stack.reserve(16);
}

inline void Tokenizer::allowAsciiType(bool allow) {
  allow_ascii_properties = allow;
}

inline void Tokenizer::allowNewLineAsTokenDelimiter(bool allow) {
  allow_new_lines = allow;
}

inline void Tokenizer::allowSuperfluousComma(bool allow) {
  allow_superfluous_comma = allow;
}
inline void Tokenizer::addData(const char *data, size_t data_size) {
  data_list.push_back(DataRef(data, data_size));
}

template <size_t N>
inline void Tokenizer::addData(const char (&data)[N]) {
  data_list.push_back(DataRef(data));
}

inline void Tokenizer::addData(const std::vector<Token> *parsedData) {
  assert(parsed_data_vector == 0);
  parsed_data_vector = parsedData;
  cursor_index = 0;
}

inline void Tokenizer::resetData(const char *data, size_t size, size_t index) {
  for (auto &data_buffer : data_list)
    release_callbacks.invokeCallbacks(data_buffer.data);
  data_list.clear();
  parsed_data_vector = nullptr;
  cursor_index = index;
  addData(data, size);
  resetForNewToken();
}

inline void Tokenizer::resetData(const std::vector<Token> *parsedData, size_t index) {
  for (auto &data_buffer : data_list)
    release_callbacks.invokeCallbacks(data_buffer.data);
  data_list.clear();
  parsed_data_vector = parsedData;
  cursor_index = index;
  resetForNewToken();
}

inline size_t Tokenizer::registeredBuffers() const {
  return data_list.size();
}

inline NeedMoreDataCBRef Tokenizer::registerNeedMoreDataCallback(std::function<void(Tokenizer &)> callback) {
  return need_more_data_callbacks.addCallback(callback);
}

inline ReleaseCBRef Tokenizer::registerReleaseCallback(std::function<void(const char *)> &callback) {
  return release_callbacks.addCallback(callback);
}

inline Error Tokenizer::nextToken(Token &next_token) {
  assert(!scope_counter.size() ||
         (scope_counter.back().type != JS::Type::ArrayEnd && scope_counter.back().type != JS::Type::ObjectEnd));
  if (scope_counter.size() && scope_counter.back().depth == 0) {
    return Error::ScopeHasEnded;
  }
  if (parsed_data_vector) {
    next_token = (*parsed_data_vector)[cursor_index];
    cursor_index++;
    if (cursor_index == parsed_data_vector->size()) {
      cursor_index = 0;
      parsed_data_vector = nullptr;
    }
    if (scope_counter.size())
      scope_counter.back().handleType(next_token.value_type);
    return Error::NoError;
  }
  if (data_list.empty()) {
    requestMoreData();
  }

  error_context.clear();

  if (data_list.empty()) {
    return Error::NeedMoreData;
  }

  if (!continue_after_need_more_data)
    resetForNewToken();

  Error error = Error::NeedMoreData;
  while (error == Error::NeedMoreData && data_list.size()) {
    const DataRef &json_data = data_list.front();
    error = populateNextTokenFromDataRef(next_token, json_data);

    if (error != Error::NoError && error != Error::NeedMoreData)
      updateErrorContext(error);

    if (error == Error::NeedMoreData) {
      releaseFirstDataRef();
      requestMoreData();
    }
  }

  continue_after_need_more_data = error == Error::NeedMoreData;
  if (error == JS::Error::NoError) {
    if (next_token.value_type == Type::ArrayStart || next_token.value_type == Type::ObjectStart)
      container_stack.push_back(next_token.value_type);
    if (next_token.value_type == Type::ArrayEnd) {
      assert(container_stack.size() && container_stack.back() == JS::Type::ArrayStart);
      container_stack.pop_back();
    }
    if (next_token.value_type == Type::ObjectEnd) {
      assert(container_stack.size() && container_stack.back() == JS::Type::ObjectStart);
      container_stack.pop_back();
    }
    if (scope_counter.size())
      scope_counter.back().handleType(next_token.value_type);
  }
  return error;
}

inline const char *Tokenizer::currentPosition() const {
  if (parsed_data_vector)
    return reinterpret_cast<const char *>(cursor_index);

  if (data_list.empty())
    return nullptr;

  return data_list.front().data + cursor_index;
}

static bool isValueInIntermediateToken(const Token &token, const Internal::IntermediateToken &intermediate) {
  if (intermediate.data.size())
    return token.value.data >= &intermediate.data[0] &&
           token.value.data < &intermediate.data[0] + intermediate.data.size();
  return false;
}

inline void Tokenizer::copyFromValue(const Token &token, std::string &to_buffer) {
  if (isValueInIntermediateToken(token, intermediate_token)) {
    std::string data(token.value.data, token.value.size);
    to_buffer += data;
    auto pair = std::make_pair(cursor_index, &to_buffer);
    copy_buffers.push_back(pair);
  } else {
    assert(token.value.data >= data_list.front().data &&
           token.value.data < data_list.front().data + data_list.front().size);
    ptrdiff_t index = token.value.data - data_list.front().data;
    auto pair = std::make_pair(index, &to_buffer);
    copy_buffers.push_back(pair);
  }
}

inline void Tokenizer::copyIncludingValue(const Token &, std::string &to_buffer) {
  auto it =
      std::find_if(copy_buffers.begin(), copy_buffers.end(),
                   [&to_buffer](const std::pair<size_t, std::string *> &pair) { return &to_buffer == pair.second; });
  assert(it != copy_buffers.end());
  assert(it->first <= cursor_index);
  if (cursor_index - it->first != 0)
    to_buffer.append(data_list.front().data + it->first, cursor_index - it->first);
  copy_buffers.erase(it);
}

inline void Tokenizer::pushScope(JS::Type type) {
  scope_counter.push_back({type, 1});
  if (type != Type::ArrayStart && type != Type::ObjectStart)
    scope_counter.back().depth--;
}

inline void Tokenizer::popScope() {
  assert(scope_counter.size() && scope_counter.back().depth == 0);
  scope_counter.pop_back();
}

inline JS::Error Tokenizer::goToEndOfScope(JS::Token &token) {
  JS::Error error = JS::Error::NoError;
  while (scope_counter.back().depth && error == JS::Error::NoError) {
    error = nextToken(token);
  }
  return error;
}

namespace Internal {
static const char *error_strings[] = {
    "NoError",
    "NeedMoreData",
    "InvalidToken",
    "ExpectedPropertyName",
    "ExpectedDelimiter",
    "ExpectedDataToken",
    "ExpectedObjectStart",
    "ExpectedObjectEnd",
    "ExpectedArrayStart",
    "ExpectedArrayEnd",
    "IllegalPropertyName",
    "IllegalPropertyType",
    "IllegalDataValue",
    "EncounteredIllegalChar",
    "NodeNotFound",
    "MissingPropertyMember",
    "MissingFunction",
    "FailedToParseBoolean",
    "FailedToParseDouble",
    "FailedToParseFloat",
    "FailedToParseInt",
    "UnassignedRequiredMember",
    "NonContigiousMemory",
    "ScopeHasEnded",
    "KeyNotFound",
    "DuplicateInSet",
    "UnknownError",
    "UserDefinedErrors",
};
}

inline std::string Tokenizer::makeErrorString() const {
  static_assert(sizeof(Internal::error_strings) / sizeof *Internal::error_strings ==
                    size_t(Error::UserDefinedErrors) + 1,
                "Please add missing error message");

  std::string retString("Error");
  if (error_context.error < Error::UserDefinedErrors)
    retString += std::string(" ") + Internal::error_strings[int(error_context.error)];
  if (error_context.custom_message.size())
    retString += " " + error_context.custom_message;
  retString += std::string(":\n");
  for (size_t i = 0; i < error_context.lines.size(); i++) {
    retString += error_context.lines[i] + "\n";
    if (i == error_context.line) {
      std::string pointing(error_context.character + 2, ' ');
      pointing[error_context.character] = '^';
      pointing[error_context.character + 1] = '\n';
      retString += pointing;
    }
  }
  return retString;
}

inline void Tokenizer::setErrorContextConfig(size_t lineContext, size_t rangeContext) {
  line_context = lineContext;
  range_context = rangeContext;
}

inline void Tokenizer::resetForNewToken() {
  intermediate_token.clear();
  resetForNewValue();
}

inline void Tokenizer::resetForNewValue() {
  property_state = InPropertyState::NoStartFound;
  property_type = Type::Error;
  current_data_start = 0;
}

inline Error Tokenizer::findStringEnd(const DataRef &json_data, size_t *chars_ahead) {
  size_t end = cursor_index;
  while (end < json_data.size) {
    if (is_escaped) {
      is_escaped = false;
      end++;
      continue;
    }
    while (end + 4 < json_data.size) {
      unsigned char lc = Internal::lookup()[(unsigned char)json_data.data[end]];
      if (lc == Internal::StrEndOrBackSlash)
        break;
      lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
      if (lc == Internal::StrEndOrBackSlash)
        break;
      lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
      if (lc == Internal::StrEndOrBackSlash)
        break;
      lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
      if (lc == Internal::StrEndOrBackSlash)
        break;
      end++;
    }
    if (end >= json_data.size)
      break;
    char c = json_data.data[end];
    if (c == '\\') {
      is_escaped = true;
    } else if (c == '"') {
      *chars_ahead = end + 1 - cursor_index;
      return Error::NoError;
    }
    end++;
  }
  return Error::NeedMoreData;
}

inline Error Tokenizer::findAsciiEnd(const DataRef &json_data, size_t *chars_ahead) {
  assert(property_type == Type::Ascii);
  size_t end = cursor_index;
  while (end < json_data.size) {
    while (end + 4 < json_data.size) {
      unsigned char lc = Internal::lookup()[(unsigned char)json_data.data[end]];
      if (!(lc & (Internal::AsciiLetters | Internal::Digits | Internal::HatUnderscoreAprostoph)))
        break;
      lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
      if (!(lc & (Internal::AsciiLetters | Internal::Digits | Internal::HatUnderscoreAprostoph)))
        break;
      lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
      if (!(lc & (Internal::AsciiLetters | Internal::Digits | Internal::HatUnderscoreAprostoph)))
        break;
      lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
      if (!(lc & (Internal::AsciiLetters | Internal::Digits | Internal::HatUnderscoreAprostoph)))
        break;
      end++;
    }

    char ascii_code = json_data.data[end];
    if ((ascii_code >= 'A' && ascii_code <= 'Z') || (ascii_code >= '^' && ascii_code <= 'z') ||
        (ascii_code >= '0' && ascii_code <= '9')) {
      end++;
      continue;
    } else if (ascii_code == '\0') {
      *chars_ahead = end - cursor_index;
      return Error::NeedMoreData;
    } else {
      *chars_ahead = end - cursor_index;
      return Error::NoError;
    }
  }
  return Error::NeedMoreData;
}

inline Error Tokenizer::findNumberEnd(const DataRef &json_data, size_t *chars_ahead) {
  size_t end = cursor_index;
  while (end + 4 < json_data.size) {
    unsigned char lc = Internal::lookup()[(unsigned char)json_data.data[end]];
    if (!(lc & (Internal::NumberEnd)))
      break;
    lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
    if (!(lc & (Internal::NumberEnd)))
      break;
    lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
    if (!(lc & (Internal::NumberEnd)))
      break;
    lc = Internal::lookup()[(unsigned char)json_data.data[++end]];
    if (!(lc & (Internal::NumberEnd)))
      break;
    end++;
  }
  while (end < json_data.size) {
    unsigned char lc = Internal::lookup()[(unsigned char)json_data.data[end]];
    if (lc & (Internal::NumberEnd)) {
      end++;
    } else {
      *chars_ahead = end - cursor_index;
      return Error::NoError;
    }
  }
  return Error::NeedMoreData;
}

inline Error Tokenizer::findStartOfNextValue(Type *type, const DataRef &json_data, size_t *chars_ahead) {
  assert(property_state == InPropertyState::NoStartFound);

  for (size_t current_pos = cursor_index; current_pos < json_data.size; current_pos++) {
    const char c = json_data.data[current_pos];
    unsigned char lc = Internal::lookup()[(unsigned char)c];
    if (c == '"') {
      *type = Type::String;
      *chars_ahead = current_pos - cursor_index;
      return Error::NoError;
    } else if (c == '{') {
      *type = Type::ObjectStart;
      *chars_ahead = current_pos - cursor_index;
      return Error::NoError;
    } else if (c == '}') {
      *type = Type::ObjectEnd;
      *chars_ahead = current_pos - cursor_index;
      return Error::NoError;
    } else if (c == '[') {
      *type = Type::ArrayStart;
      *chars_ahead = current_pos - cursor_index;
      return Error::NoError;
    } else if (c == ']') {
      *type = Type::ArrayEnd;
      *chars_ahead = current_pos - cursor_index;
      return Error::NoError;
    } else if (lc & (Internal::PlusOrMinus | Internal::Digits)) {
      *type = Type::Number;
      *chars_ahead = current_pos - cursor_index;
      return Error::NoError;
    } else if (lc & Internal::AsciiLetters) {
      *type = Type::Ascii;
      *chars_ahead = current_pos - cursor_index;
      return Error::NoError;
    } else if (lc == 0) {
      *chars_ahead = current_pos - cursor_index;
      return Error::EncounteredIllegalChar;
    }
  }
  return Error::NeedMoreData;
}

inline Error Tokenizer::findDelimiter(const DataRef &json_data, size_t *chars_ahead) {
  if (container_stack.empty())
    return Error::IllegalPropertyType;
  for (size_t end = cursor_index; end < json_data.size; end++) {
    const char c = json_data.data[end];
    if (c == ':') {
      if (container_stack.back() != Type::ObjectStart)
        return Error::ExpectedDelimiter;
      token_state = InTokenState::FindingData;
      *chars_ahead = end + 1 - cursor_index;
      return Error::NoError;
    } else if (c == ',') {
      if (container_stack.back() != Type::ArrayStart)
        return Error::ExpectedDelimiter;
      token_state = InTokenState::FindingName;
      *chars_ahead = end + 1 - cursor_index;
      return Error::NoError;
    } else if (c == ']') {
      if (container_stack.back() != Type::ArrayStart)
        return Error::ExpectedDelimiter;
      token_state = InTokenState::FindingName;
      *chars_ahead = end - cursor_index;
      return Error::NoError;
    } else if (!(Internal::lookup()[(unsigned char)c] & Internal::WhiteSpaceOrNull)) {
      return Error::ExpectedDelimiter;
    }
  }
  return Error::NeedMoreData;
}

inline Error Tokenizer::findTokenEnd(const DataRef &json_data, size_t *chars_ahead) {
  if (container_stack.empty())
    return Error::NoError;
  for (size_t end = cursor_index; end < json_data.size; end++) {
    const char c = json_data.data[end];
    if (c == ',') {
      expecting_prop_or_annonymous_data = true;
      *chars_ahead = end + 1 - cursor_index;
      return Error::NoError;
    } else if (c == ']' || c == '}') {
      *chars_ahead = end - cursor_index;
      return Error::NoError;
    } else if (c == '\n') {
      if (allow_new_lines) {
        *chars_ahead = end + 1 - cursor_index;
        return Error::NoError;
      }
    } else if (Internal::lookup()[(unsigned char)c] & Internal::WhiteSpaceOrNull) {
      continue;
    } else {
      *chars_ahead = end + 1 - cursor_index;
      return Error::InvalidToken;
    }
  }
  return Error::NeedMoreData;
}

inline void Tokenizer::requestMoreData() {
  need_more_data_callbacks.invokeCallbacks(*this);
}

inline void Tokenizer::releaseFirstDataRef() {
  if (data_list.empty())
    return;

  const DataRef &json_data = data_list.front();

  for (auto &copy_pair : copy_buffers) {
    std::string data(json_data.data + copy_pair.first, json_data.size - copy_pair.first);
    *copy_pair.second += data;
    copy_pair.first = 0;
  }

  cursor_index = 0;
  current_data_start = 0;

  const char *data_to_release = json_data.data;
  data_list.erase(data_list.begin());
  release_callbacks.invokeCallbacks(data_to_release);
}

inline Error Tokenizer::populateFromDataRef(DataRef &data, Type &type, const DataRef &json_data) {
  size_t diff = 0;
  Error error = Error::NoError;
  data.size = 0;
  data.data = json_data.data + cursor_index;
  if (property_state == InPropertyState::NoStartFound) {
    error = findStartOfNextValue(&type, json_data, &diff);
    if (error != Error::NoError) {
      type = Type::Error;
      return error;
    }

    data.data = json_data.data + cursor_index + diff;
    current_data_start = cursor_index + diff;
    if (type == Type::String) {
      data.data++;
      current_data_start++;
    }
    cursor_index += diff + 1;
    property_type = type;

    if (type == Type::ObjectStart || type == Type::ObjectEnd || type == Type::ArrayStart || type == Type::ArrayEnd) {
      data.size = 1;
      property_state = InPropertyState::FoundEnd;
    } else {
      property_state = InPropertyState::FindingEnd;
    }
  }

  size_t negative_size_adjustment = 0;
  if (property_state == InPropertyState::FindingEnd) {
    switch (type) {
      case Type::String:
        error = findStringEnd(json_data, &diff);
        negative_size_adjustment = 1;
        break;
      case Type::Ascii:
        error = findAsciiEnd(json_data, &diff);
        break;
      case Type::Number:
        error = findNumberEnd(json_data, &diff);
        break;
      default:
        return Error::InvalidToken;
    }

    if (error != Error::NoError) {
      return error;
    }

    cursor_index += diff;
    data.size = cursor_index - current_data_start - negative_size_adjustment;
    property_state = InPropertyState::FoundEnd;
  }

  return Error::NoError;
}

inline void Tokenizer::populate_annonymous_token(const DataRef &data, Type type, Token &token) {
  token.name = DataRef();
  token.name_type = Type::Ascii;
  token.value = data;
  token.value_type = type;
}

namespace Internal {
static Type getType(Type type, const char *data, size_t length) {
  static const char m_null[] = "null";
  static const char m_true[] = "true";
  static const char m_false[] = "false";
  if (type != Type::Ascii)
    return type;
  if (sizeof(m_null) - 1 == length) {
    if (memcmp(m_null, data, length) == 0) {
      return Type::Null;
    } else if (memcmp(m_true, data, length) == 0) {
      return Type::Bool;
    }
  }
  if (sizeof(m_false) - 1 == length) {
    if (memcmp(m_false, data, length) == 0)
      return Type::Bool;
  }
  return Type::Ascii;
}

inline size_t strnlen(const char *data, size_t size) {
  auto it = std::find(data, data + size, '\0');
  return it - data;
}

}  // namespace Internal

inline Error Tokenizer::populateNextTokenFromDataRef(Token &next_token, const DataRef &json_data) {
  Token tmp_token;
  while (cursor_index < json_data.size) {
    size_t diff = 0;
    DataRef data;
    Type type;
    Error error;
    switch (token_state) {
      case InTokenState::FindingName:
        type = intermediate_token.name_type;
        error = populateFromDataRef(data, type, json_data);
        if (error == Error::NeedMoreData) {
          if (property_state > InPropertyState::NoStartFound) {
            intermediate_token.active = true;
            size_t to_null = Internal::strnlen(data.data, json_data.size - current_data_start);
            intermediate_token.name.append(data.data, to_null);
            if (!intermediate_token.name_type_set) {
              intermediate_token.name_type = type;
              intermediate_token.name_type_set = true;
            }
          }
          return error;
        } else if (error != Error::NoError) {
          return error;
        }

        if (intermediate_token.active) {
          intermediate_token.name.append(data.data, data.size);
          data = DataRef(intermediate_token.name);
          type = intermediate_token.name_type;
        }

        if (type == Type::ObjectEnd || type == Type::ArrayEnd || type == Type::ArrayStart || type == Type::ObjectStart) {
          switch (type) {
            case Type::ObjectEnd:
            case Type::ArrayEnd:
              if (expecting_prop_or_annonymous_data && !allow_superfluous_comma) {
                return Error::ExpectedDataToken;
              }
              populate_annonymous_token(data, type, next_token);
              token_state = InTokenState::FindingTokenEnd;
              return Error::NoError;

            case Type::ObjectStart:
            case Type::ArrayStart:
              populate_annonymous_token(data, type, next_token);
              expecting_prop_or_annonymous_data = false;
              token_state = InTokenState::FindingName;
              return Error::NoError;
            default:
              return Error::UnknownError;
          }
        } else {
          tmp_token.name = data;
        }

        tmp_token.name_type = Internal::getType(type, tmp_token.name.data, tmp_token.name.size);
        token_state = InTokenState::FindingDelimiter;
        resetForNewValue();
        break;

      case InTokenState::FindingDelimiter:
        error = findDelimiter(json_data, &diff);
        if (error != Error::NoError) {
          if (intermediate_token.active == false) {
            intermediate_token.name.append(tmp_token.name.data, tmp_token.name.size);
            intermediate_token.name_type = tmp_token.name_type;
            intermediate_token.active = true;
          }
          return error;
        }
        cursor_index += diff;
        resetForNewValue();
        expecting_prop_or_annonymous_data = false;
        if (token_state == InTokenState::FindingName) {
          populate_annonymous_token(tmp_token.name, tmp_token.name_type, next_token);
          return Error::NoError;
        } else {
          if (tmp_token.name_type != Type::String) {
            if (!allow_ascii_properties || tmp_token.name_type != Type::Ascii) {
              return Error::IllegalPropertyName;
            }
          }
        }
        break;

      case InTokenState::FindingData:
        type = intermediate_token.data_type;
        error = populateFromDataRef(data, type, json_data);
        if (error == Error::NeedMoreData) {
          if (intermediate_token.active == false) {
            intermediate_token.name.append(tmp_token.name.data, tmp_token.name.size);
            intermediate_token.name_type = tmp_token.name_type;
            intermediate_token.active = true;
          }
          if (property_state > InPropertyState::NoStartFound) {
            size_t data_length = Internal::strnlen(data.data, json_data.size - current_data_start);
            intermediate_token.data.append(data.data, data_length);
            if (!intermediate_token.data_type_set) {
              intermediate_token.data_type = type;
              intermediate_token.data_type_set = true;
            }
          }
          return error;
        } else if (error != Error::NoError) {
          return error;
        }

        if (intermediate_token.active) {
          intermediate_token.data.append(data.data, data.size);
          if (!intermediate_token.data_type_set) {
            intermediate_token.data_type = type;
            intermediate_token.data_type_set = true;
          }
          tmp_token.name = DataRef(intermediate_token.name);
          tmp_token.name_type = intermediate_token.name_type;
          data = DataRef(intermediate_token.data);
          type = intermediate_token.data_type;
        }

        tmp_token.value = data;
        tmp_token.value_type = Internal::getType(type, tmp_token.value.data, tmp_token.value.size);

        if (tmp_token.value_type == Type::Ascii && !allow_ascii_properties)
          return Error::IllegalDataValue;

        if (type == Type::ObjectStart || type == Type::ArrayStart) {
          token_state = InTokenState::FindingName;
        } else {
          token_state = InTokenState::FindingTokenEnd;
        }
        next_token = tmp_token;
        return Error::NoError;
      case InTokenState::FindingTokenEnd:
        error = findTokenEnd(json_data, &diff);
        if (error != Error::NoError) {
          return error;
        }
        cursor_index += diff;
        token_state = InTokenState::FindingName;
        break;
    }
  }
  return Error::NeedMoreData;
}

namespace Internal {
struct Lines {
  size_t start;
  size_t end;
};
}  // namespace Internal

inline Error Tokenizer::updateErrorContext(Error error, const std::string &custom_message) {
  error_context.error = error;
  error_context.custom_message = custom_message;
  if ((!parsed_data_vector || parsed_data_vector->empty()) && data_list.empty())
    return error;

  const DataRef json_data =
      parsed_data_vector && parsed_data_vector->size()
          ? DataRef(parsed_data_vector->front().value.data,
                    size_t(parsed_data_vector->back().value.data - parsed_data_vector->front().value.data))
          : data_list.front();
  int64_t real_cursor_index = parsed_data_vector && parsed_data_vector->size()
                                  ? int64_t(parsed_data_vector->at(cursor_index).value.data - json_data.data)
                                  : int64_t(cursor_index);
  const int64_t stop_back = real_cursor_index - std::min(int64_t(real_cursor_index), int64_t(line_range_context));
  const int64_t stop_forward = std::min(real_cursor_index + int64_t(line_range_context), int64_t(json_data.size));
  std::vector<Internal::Lines> lines;
  lines.push_back({0, size_t(real_cursor_index)});
  assert(real_cursor_index <= int64_t(json_data.size));
  int64_t lines_back = 0;
  int64_t lines_forward = 0;
  int64_t cursor_back;
  int64_t cursor_forward;
  for (cursor_back = real_cursor_index - 1; cursor_back > stop_back; cursor_back--) {
    if (*(json_data.data + cursor_back) == '\n') {
      lines.front().start = size_t(cursor_back + 1);
      lines_back++;
      if (lines_back == 1)
        error_context.character = size_t(real_cursor_index - cursor_back);
      if (lines_back == int64_t(line_context)) {
        lines_back--;
        break;
      }

      lines.insert(lines.begin(), {0, size_t(cursor_back)});
    }
  }
  if (lines.front().start == 0)
    lines.front().start = size_t(cursor_back);
  bool add_new_line = false;
  for (cursor_forward = real_cursor_index; cursor_forward < stop_forward; cursor_forward++) {
    if (add_new_line) {
      lines.push_back({size_t(cursor_forward), 0});
      add_new_line = false;
    }
    if (*(json_data.data + cursor_forward) == '\n') {
      lines.back().end = size_t(cursor_forward);
      lines_forward++;
      if (lines_forward == int64_t(line_context))
        break;
      add_new_line = true;
    }
  }
  if (lines.back().end == 0)
    lines.back().end = size_t(cursor_forward - 1);

  if (lines.size() > 1) {
    error_context.lines.reserve(lines.size());
    for (auto &line : lines) {
      error_context.lines.push_back(std::string(json_data.data + line.start, line.end - line.start));
    }
    error_context.line = size_t(lines_back);
  } else {
    error_context.line = 0;

    int64_t left = real_cursor_index > int64_t(range_context) ? real_cursor_index - int64_t(range_context) : 0;
    int64_t right =
        real_cursor_index + int64_t(range_context) > int64_t(json_data.size) ? int64_t(json_data.size) : real_cursor_index + int64_t(range_context);
    error_context.character = size_t(real_cursor_index - left);
    error_context.lines.push_back(std::string(json_data.data + left, size_t(right - left)));
  }
  return error;
}

static inline JS::Error reformat(const char *data, size_t size, std::string &out,
                                 const SerializerOptions &options = SerializerOptions()) {
  Token token;
  Tokenizer tokenizer;
  tokenizer.addData(data, size);
  Error error = Error::NoError;

  Serializer serializer;
  serializer.setOptions(options);
  size_t last_pos = 0;
  auto cbref = serializer.addRequestBufferCallback([&out, &last_pos](Serializer &serializer_p) {
    size_t end = out.size();
    out.resize(end * 2);
    serializer_p.setBuffer(&out[0] + end, end);
    last_pos = end;
  });
  if (out.empty())
    out.resize(4096);
  serializer.setBuffer(&out[0], out.size());

  while (true) {
    error = tokenizer.nextToken(token);
    if (error != Error::NoError)
      break;
    serializer.write(token);
  }
  out.resize(last_pos + serializer.currentBuffer().used);
  if (error == Error::NeedMoreData)
    return Error::NoError;

  return error;
}
static inline JS::Error reformat(const std::string &in, std::string &out,
                                 const SerializerOptions &options = SerializerOptions()) {
  return reformat(in.c_str(), in.size(), out, options);
}

// Tuple start
namespace Internal {
template <size_t...>
struct Sequence {
  using type = Sequence;
};

template <typename A, typename B>
struct Merge;
template <size_t... Is1, size_t... Is2>
struct Merge<Sequence<Is1...>, Sequence<Is2...>> {
  using type = Sequence<Is1..., (sizeof...(Is1) + Is2)...>;
};

template <size_t size>
struct GenSequence;
template <>
struct GenSequence<0> {
  using type = Sequence<>;
};
template <>
struct GenSequence<1> {
  using type = Sequence<0>;
};
template <size_t size>
struct GenSequence {
  using type = typename Merge<typename GenSequence<size / size_t(2)>::type,
                              typename GenSequence<size - size / size_t(2)>::type>::type;
};

template <size_t index, typename T>
struct Element {
  constexpr Element()
      : data() {
  }

  constexpr Element(const T &t)
      : data(t) {
  }
  using type = T;
  T data;
};

template <typename A, typename... Bs>
struct TupleImpl;

template <size_t... indices, typename... Ts>
struct TupleImpl<Sequence<indices...>, Ts...> : public Element<indices, Ts>... {
  constexpr TupleImpl()
      : Element<indices, Ts>()... {
  }

  constexpr TupleImpl(Ts... args)
      : Element<indices, Ts>(args)... {
  }
};
}  // namespace Internal

template <size_t I, typename... Ts>
struct TypeAt {
  template <typename T>
  static Internal::Element<I, T> deduce(Internal::Element<I, T>);

  using tuple_impl = Internal::TupleImpl<typename Internal::GenSequence<sizeof...(Ts)>::type, Ts...>;
  using element = decltype(deduce(tuple_impl()));
  using type = typename element::type;
};

template <typename... Ts>
struct Tuple {
  constexpr Tuple()
      : impl() {
  }

  constexpr Tuple(Ts... args)
      : impl(args...) {
  }

  using Seq = typename Internal::GenSequence<sizeof...(Ts)>::type;
  Internal::TupleImpl<Seq, Ts...> impl;
  static constexpr const size_t size = sizeof...(Ts);

  template <size_t Index>
  constexpr const typename TypeAt<Index, Ts...>::type &get() const {
    return static_cast<const typename TypeAt<Index, Ts...>::element &>(impl).data;
  }

  template <size_t Index>
  typename TypeAt<Index, Ts...>::type &get() {
    return static_cast<typename TypeAt<Index, Ts...>::element &>(impl).data;
  }
};

/// \private
template <size_t I, typename... Ts>
struct TypeAt<I, const Tuple<Ts...>> {
  template <typename T>
  static Internal::Element<I, T> deduce(Internal::Element<I, T>);

  using tuple_impl = Internal::TupleImpl<typename Internal::GenSequence<sizeof...(Ts)>::type, Ts...>;
  using element = decltype(deduce(tuple_impl()));
  using type = typename element::type;
};

/// \private
template <size_t I, typename... Ts>
struct TypeAt<I, Tuple<Ts...>> {
  template <typename T>
  static Internal::Element<I, T> deduce(Internal::Element<I, T>);

  using tuple_impl = Internal::TupleImpl<typename Internal::GenSequence<sizeof...(Ts)>::type, Ts...>;
  using element = decltype(deduce(tuple_impl()));
  using type = typename element::type;
};

/*!  \private
 */
template <>
struct Tuple<> {
  static constexpr const size_t size = 0;
};

template <typename... Ts>
constexpr Tuple<Ts...> makeTuple(Ts... args) {
  return Tuple<Ts...>(args...);
}
// Tuple end

inline SerializerOptions::SerializerOptions(Style style)

    : m_shift_size(style == Compact ? 0 : 2), m_depth(0), m_style(style), m_convert_ascii_to_string(true), m_token_delimiter(","), m_value_delimiter(style == Pretty ? ": " : ":"), m_postfix(style == Pretty ? "\n" : "") {
}

inline int SerializerOptions::shiftSize() const {
  return m_shift_size;
}

inline void SerializerOptions::setShiftSize(unsigned char set) {
  m_shift_size = set;
}

inline unsigned char SerializerOptions::depth() const {
  return m_depth;
}

inline SerializerOptions::Style SerializerOptions::style() const {
  return m_style;
}

inline bool SerializerOptions::convertAsciiToString() const {
  return m_convert_ascii_to_string;
}

inline void SerializerOptions::setConvertAsciiToString(bool set) {
  m_convert_ascii_to_string = set;
}

inline void SerializerOptions::setStyle(Style style) {
  m_style = style;
  m_postfix = m_style == Pretty ? std::string("\n") : std::string("");
  m_value_delimiter = m_style == Pretty ? std::string(" : ") : std::string(":");
  setDepth(m_depth);
}

inline void SerializerOptions::skipDelimiter(bool skip) {
  if (skip)
    m_token_delimiter = "";
  else
    m_token_delimiter = ",";
}

inline void SerializerOptions::setDepth(int depth) {
  m_depth = (unsigned char)depth;
  m_prefix = m_style == Pretty ? std::string(depth * size_t(m_shift_size), ' ') : std::string();
}

inline const std::string &SerializerOptions::prefix() const {
  return m_prefix;
}
inline const std::string &SerializerOptions::tokenDelimiter() const {
  return m_token_delimiter;
}
inline const std::string &SerializerOptions::valueDelimiter() const {
  return m_value_delimiter;
}
inline const std::string &SerializerOptions::postfix() const {
  return m_postfix;
}

inline void SerializerBuffer::append(const char *data, size_t data_size) {
  assert(used + data_size <= size);
  memcpy(buffer + used, data, data_size);
  used += data_size;
}

template <size_t SIZE>
inline void SerializerBuffer::append(const char *data) {
  assert(used + SIZE <= size);
  memcpy(buffer + used, data, SIZE);
  used += SIZE;
}

inline Serializer::Serializer()
    : m_first(true), m_token_start(true) {
}

inline Serializer::Serializer(char *buffer, size_t size)
    : m_current_buffer(buffer, size), m_first(true), m_token_start(true)

{
}

inline void Serializer::setBuffer(char *buffer, size_t size) {
  m_current_buffer = SerializerBuffer(buffer, size);
}

inline void Serializer::setOptions(const SerializerOptions &option) {
  m_option = option;
}

inline bool Serializer::write(const Token &in_token) {
  auto begining_literals = makeTuple(JS::Internal::makeStringLiteral("\n  "),
                                     Internal::makeStringLiteral("\n    "),
                                     Internal::makeStringLiteral("\n      "),
                                     Internal::makeStringLiteral("\n        "),
                                     Internal::makeStringLiteral("\n          "),
                                     Internal::makeStringLiteral(",\n  "),
                                     Internal::makeStringLiteral(",\n    "),
                                     Internal::makeStringLiteral(",\n      "),
                                     Internal::makeStringLiteral(",\n        "),
                                     Internal::makeStringLiteral(",\n          "));
  //auto begining_literals_compat = makeTuple( Internal::makeStringLiteral(",\""));
  const Token &token = in_token;

  bool isEnd = token.value_type == Type::ObjectEnd || token.value_type == Type::ArrayEnd;
  if (isEnd) {
    assert(m_option.depth() > 0);
    m_option.setDepth(m_option.depth() - 1);
  }

  bool shortcut_front = false;
  if (m_option.shiftSize() == 2 && !m_first) {
    if (!m_token_start && !isEnd) {
      if (m_option.depth() == 1)
        shortcut_front = write(begining_literals.get<5>());
      else if (m_option.depth() == 2)
        shortcut_front = write(begining_literals.get<6>());
      else if (m_option.depth() == 3)
        shortcut_front = write(begining_literals.get<7>());
      else if (m_option.depth() == 4)
        shortcut_front = write(begining_literals.get<8>());
      else if (m_option.depth() == 5)
        shortcut_front = write(begining_literals.get<9>());
    } else {
      if (m_option.depth() == 1)
        shortcut_front = write(begining_literals.get<0>());
      else if (m_option.depth() == 2)
        shortcut_front = write(begining_literals.get<1>());
      else if (m_option.depth() == 3)
        shortcut_front = write(begining_literals.get<2>());
      else if (m_option.depth() == 4)
        shortcut_front = write(begining_literals.get<3>());
      else if (m_option.depth() == 5)
        shortcut_front = write(begining_literals.get<4>());
    }
  }

  if (!shortcut_front) {
    if (!m_token_start) {
      if (!isEnd) {
        if (!m_option.tokenDelimiter().empty()) {
          if (!write(Internal::makeStringLiteral(",")))
            return false;
        }
      }
    }

    if (m_first) {
      m_first = false;
    } else {
      if (!m_option.postfix().empty())
        if (!write(m_option.postfix()))
          return false;
    }

    if (!m_option.prefix().empty())
      if (!write(m_option.prefix()))
        return false;
  }
  if (token.name.size) {
    if (!write(token.name_type, token.name))
      return false;

    if (m_option.style() == SerializerOptions::Pretty) {
      if (!write(Internal::makeStringLiteral(": ")))
        return false;
    } else {
      if (!write(Internal::makeStringLiteral(":")))
        return false;
    }
  }

  if (!write(token.value_type, token.value))
    return false;

  m_token_start = (token.value_type == Type::ObjectStart || token.value_type == Type::ArrayStart);
  if (m_token_start) {
    m_option.setDepth(m_option.depth() + 1);
  }
  return true;
}

inline const BufferRequestCBRef Serializer::addRequestBufferCallback(std::function<void(Serializer &)> callback) {
  return m_request_buffer_callbacks.addCallback(callback);
}

inline const SerializerBuffer &Serializer::currentBuffer() const {
  return m_current_buffer;
}

inline void Serializer::askForMoreBuffers() {
  m_request_buffer_callbacks.invokeCallbacks(*this);
}

inline void Serializer::markCurrentSerializerBufferFull() {
  m_current_buffer = SerializerBuffer();
  askForMoreBuffers();
}

inline bool Serializer::writeAsString(const DataRef &data) {
  bool written;
  written = write(Internal::makeStringLiteral("\""));
  if (!written)
    return false;

  written = write(data.data, data.size);
  if (!written)
    return false;

  written = write(Internal::makeStringLiteral("\""));

  return written;
}

inline bool Serializer::write(Type type, const DataRef &data) {
  bool written;
  switch (type) {
    case Type::String:
      written = writeAsString(data);
      break;
    case Type::Ascii:
      if (m_option.convertAsciiToString())
        written = writeAsString(data);
      else
        written = write(data.data, data.size);
      break;
    case Type::Null:
      written = write("null", 4);
      break;
    default:
      written = write(data.data, data.size);
      break;
  }
  return written;
}

inline bool Serializer::write(const char *data, size_t size) {
  if (!size)
    return true;
  size_t written = 0;
  while (written < size) {
    size_t free = m_current_buffer.free();
    if (free == 0) {
      markCurrentSerializerBufferFull();
      if (!m_current_buffer.free())
        return false;
      continue;
    }
    size_t to_write = std::min(size - written, free);
    m_current_buffer.append(data + written, to_write);
    written += to_write;
  }
  return written == size;
}

template <size_t SIZE>
inline bool Serializer::write(const Internal::StringLiteral<SIZE> &strLiteral) {
  if (m_current_buffer.free() < SIZE)
    return write(strLiteral.data, SIZE);

  m_current_buffer.append<SIZE>(strLiteral.data);
  return true;
}

template <typename T>
struct Nullable {
  Nullable()
      : data() {
  }
  Nullable(const T &t)
      : data(t) {
  }
  Nullable(T &&t)
      : data(std::move(t)) {
  }

  Nullable(Nullable<T> &&t)
      : data(std::move(t.data)) {
  }
  Nullable(const Nullable<T> &t)
      : data(t.data) {
  }

  Nullable<T> &operator=(const T &other) {
    data = other;
    return *this;
  }
  Nullable<T> &operator=(T &&other) {
    data = std::move(other);
    return *this;
  }

  Nullable<T> &operator=(const Nullable<T> &other) {
    data = other.data;
    return *this;
  }
  Nullable<T> &operator=(Nullable<T> &&other) {
    data = std::move(other.data);
    return *this;
  }

  T data;
  T &operator()() {
    return data;
  }
  const T &operator()() const {
    return data;
  }
};

template <typename T>
struct NullableChecked {
  NullableChecked()
      : data(), null(true) {
  }
  NullableChecked(const T &t)
      : data(t), null(false) {
  }
  NullableChecked(T &&t)
      : data(std::move(t)), null(false) {
  }
  NullableChecked(const NullableChecked<T> &t)
      : data(t.data), null(t.null) {
  }
  NullableChecked(NullableChecked<T> &&t)
      : data(std::move(t.data)), null(t.null) {
  }
  NullableChecked<T> &operator=(const T &other) {
    data = other;
    null = false;
    return *this;
  }
  NullableChecked<T> &operator=(T &&other) {
    data = std::move(other);
    null = false;
    return *this;
  }

  NullableChecked<T> &operator=(const NullableChecked<T> &other) {
    data = other.data;
    null = other.null;
    return *this;
  }
  NullableChecked<T> &operator=(NullableChecked<T> &&other) {
    data = std::move(other.data);
    null = other.null;
    return *this;
  }

  T &operator()() {
    return data;
  }
  const T &operator()() const {
    return data;
  }
  T data;
  bool null;
};

template <typename T>
struct Optional {
  Optional()
      : data() {
  }
  Optional(const T &t)
      : data(t) {
  }
  Optional(T &&t)
      : data(std::move(t)) {
  }

  Optional(const Optional<T> &t)
      : data(t.data) {
  }
  Optional(Optional<T> &&t)
      : data(std::move(t.data)) {
  }
  Optional<T> &operator=(const T &other) {
    data = other;
    return *this;
  }

  Optional<T> &operator=(T &&other) {
    data = std::move(other);
    return *this;
  }

  Optional<T> &operator=(const Optional<T> &other) {
    data = other.data;
    return *this;
  }

  Optional<T> &operator=(Optional<T> &&other) {
    data = std::move(other.data);
    return *this;
  }

  T data;
  T &operator()() {
    return data;
  }
  const T &operator()() const {
    return data;
  }
  typedef bool IsOptionalType;
};

template <typename T>
struct OptionalChecked {
  OptionalChecked()
      : data(), assigned(false) {
  }
  OptionalChecked(const T &t)
      : data(t), assigned(true) {
  }
  OptionalChecked(T &&t)
      : data(std::move(t)), assigned(true) {
  }
  OptionalChecked(const OptionalChecked<T> &t)
      : data(t.data), assigned(t.assigned) {
  }
  OptionalChecked(OptionalChecked<T> &&t)
      : data(std::move(t.data)), assigned(t.assigned) {
  }
  OptionalChecked<T> &operator=(const T &other) {
    data = other;
    assigned = true;
    return *this;
  }
  OptionalChecked<T> &operator=(T &&other) {
    data = std::move(other);
    assigned = true;
    return *this;
  }
  OptionalChecked<T> &operator=(const OptionalChecked<T> &other) {
    data = other.data;
    assigned = other.assigned;
    return *this;
  }
  OptionalChecked<T> &operator=(OptionalChecked<T> &&other) {
    data = std::move(other.data);
    assigned = other.assigned;
    return *this;
  }

  T &operator()() {
    return data;
  }
  const T &operator()() const {
    return data;
  }
#ifdef JS_STD_OPTIONAL
  std::optional<T> opt() const {
    return assigned ? std::optional<T>(data) : std::nullopt;
  }
#endif
  T data;
  bool assigned;
  typedef bool IsOptionalType;
};

struct SilentString {
  std::string data;
  typedef bool IsOptionalType;
};

template <typename T, typename A = std::allocator<T>>
struct SilentVector {
  std::vector<T, A> data;
  typedef bool IsOptionalType;
};

template <typename T, typename Deleter = std::default_delete<T>>
struct SilentUniquePtr {
  std::unique_ptr<T, Deleter> data;
  typedef bool IsOptionalType;
};

struct JsonObjectRef {
  DataRef ref;
};

struct JsonObject {
  std::string data;
};

struct JsonArrayRef {
  DataRef ref;
};

struct JsonArray {
  std::string data;
};

struct JsonObjectOrArrayRef {
  DataRef ref;
};

struct JsonObjectOrArray {
  std::string data;
};

struct JsonTokens {
  std::vector<JS::Token> data;
};

struct JsonMeta {
  JsonMeta(size_t pos, bool is_array)
      : position(pos), size(1), skip(1), children(0), complex_children(0), is_array(is_array), has_data(false) {
  }

  size_t position;
  uint32_t size;
  uint32_t skip;
  uint32_t children;
  uint32_t complex_children;
  bool is_array : 1;
  bool has_data : 1;
};

static inline std::vector<JsonMeta> metaForTokens(const JsonTokens &tokens) {
  std::vector<JsonMeta> meta;
  meta.reserve(tokens.data.size() / 4);
  std::vector<size_t> parent;
  for (size_t i = 0; i < tokens.data.size(); i++) {
    for (size_t parent_index : parent) {
      meta[parent_index].size++;
    }
    const JS::Token &token = tokens.data.at(i);
    if (token.value_type == Type::ArrayEnd || token.value_type == Type::ObjectEnd) {
      assert(parent.size());
      assert(meta[parent.back()].is_array == (token.value_type == Type::ArrayEnd));
      parent.pop_back();
    } else {
      if (parent.size())
        meta[parent.back()].children++;
    }

    if (token.value_type == Type::ArrayStart || token.value_type == Type::ObjectStart) {
      if (parent.size())
        meta[parent.back()].complex_children++;
      for (size_t parent_index : parent) {
        meta[parent_index].skip++;
      }
      meta.push_back(JsonMeta(i, token.value_type == Type::ArrayStart));
      parent.push_back(meta.size() - 1);
    } else if (token.value_type != JS::Type::ArrayEnd && token.value_type != JS::Type::ObjectEnd) {
      for (size_t parent_index : parent) {
        meta[parent_index].has_data = true;
      }
    }
  }
  assert(!parent.size());  // This assert may be triggered when JSON is invalid (e.g. when creating a DiffContext).
  return meta;
}

namespace Internal {
static inline size_t findFirstChildWithData(const std::vector<JsonMeta> &meta_vec, size_t start_index) {
  const JsonMeta &meta = meta_vec[start_index];
  if (!meta.has_data)
    return size_t(-1);

  size_t skip_size = 0;
  for (uint32_t i = 0; i < meta.complex_children; i++) {
    auto &current_child = meta_vec[start_index + skip_size + 1];
    skip_size += current_child.skip;
    if (current_child.has_data)
      return i;
  }
  return size_t(-1);
}
}  // namespace Internal

template <typename T>
struct IsOptionalType {
  typedef char yes[1];
  typedef char no[2];

  template <typename C>
  static constexpr yes &test_in_optional(typename C::IsOptionalType *);

  template <typename>
  static constexpr no &test_in_optional(...);

  static constexpr const bool value = sizeof(test_in_optional<T>(0)) == sizeof(yes);
};

/// \private
template <typename T>
struct IsOptionalType<std::unique_ptr<T>> {
  static constexpr const bool value = true;
};

#ifdef JS_STD_OPTIONAL
/// \private
template <typename T>
struct IsOptionalType<std::optional<T>> {
  static constexpr const bool value = true;
};
#endif

struct ParseContext {
  ParseContext() {
  }
  explicit ParseContext(const char *data, size_t size) {
    tokenizer.addData(data, size);
  }

  explicit ParseContext(const char *data) {
    size_t size = strlen(data);
    tokenizer.addData(data, size);
  }

  explicit ParseContext(const std::string &data) {
    tokenizer.addData(&data[0], data.size());
  }

  template <typename T>
  explicit ParseContext(const char *data, size_t size, T &to_type) {
    tokenizer.addData(data, size);
    auto this_error = parseTo(to_type);
    (void)this_error;
  }
  template <size_t SIZE>
  explicit ParseContext(const char (&data)[SIZE]) {
    tokenizer.addData(data);
  }

  template <typename T>
  Error parseTo(T &to_type);

  Error nextToken() {
    error = tokenizer.nextToken(token);
    return error;
  }

  std::string makeErrorString() const {
    if (error == Error::MissingPropertyMember) {
      if (missing_members.size() == 0) {
        return "";
      } else if (missing_members.size() == 1) {
        return std::string("JSON Object contained member not found in C++ struct/class. JSON Object member is: ") +
               missing_members.front();
      }
      std::string member_string = missing_members.front();
      for (int i = 1; i < int(missing_members.size()); i++)
        member_string += std::string(", ") + missing_members[i];
      return std::string("JSON Object contained members not found in C++ struct/class. JSON Object members are: ") +
             member_string;
    } else if (error == Error::UnassignedRequiredMember) {
      if (unassigned_required_members.size() == 0) {
        return "";
      } else if (unassigned_required_members.size() == 1) {
        return std::string(
                   "C++ struct/class has a required member that is not present in input JSON. The unassigned "
                   "C++ member is: ") +
               unassigned_required_members.front();
      }
      std::string required_string = unassigned_required_members.front();
      for (int i = 1; i < int(unassigned_required_members.size()); i++)
        required_string += std::string(", ") + unassigned_required_members[i];
      return std::string(
                 "C++ struct/class has required members that are not present in the input JSON. The unassigned "
                 "C++ members are: ") +
             required_string;
    }
    if (tokenizer.errorContext().error == Error::NoError && error != Error::NoError) {
      std::string retString("Error:");
      if (error <= Error::UserDefinedErrors)
        retString += Internal::error_strings[int(error)];
      else
        retString += "Unknown error";
      return retString;
    }
    return tokenizer.makeErrorString();
  }

  Tokenizer tokenizer;
  Token token;
  Error error = Error::NoError;
  std::vector<std::string> missing_members;
  std::vector<std::string> unassigned_required_members;
  bool allow_missing_members = true;
  bool allow_unasigned_required_members = true;
  bool track_member_assignement_state = true;
  void *user_data = nullptr;
};

/*! \def JS_MEMBER
 *
 * Create meta information of the member with the same name as
 * the member.
 */
/*! \def JS_MEMBER_ALIASES
 *
 * Create meta information where the primary name is the same as the member and
 * the subsequent names are aliases.
 */
/*! \def JS_MEMBER_WITH_NAME
 *
 * Create meta information where the primary name is argument name, and the subsequent
 * names are aliases.
 */
/*! \def JS_MEMBER_WITH_NAME_AND_ALIASES
 *
 * Creates meta information where the primary name is argument name, a
 * and subsequent names are aliases
 */

/*! \def JS_SUPER_CLASS
 *
 * Creates superclass meta data which is used inside the JS_SUPER_CLASSES macro
 */

/*! \def JS_SUPER_CLASSES
 *
 * Macro to contain the super class definitions
 */

namespace Internal {
template <typename T>
struct HasJsonStructBase {
  typedef char yes[1];
  typedef char no[2];

  template <typename C>
  static constexpr yes &test_in_base(typename C::template JsonStructBase<C> *);

  template <typename>
  static constexpr no &test_in_base(...);
};

template <typename JS_BASE_STRUCT_T, typename JS_OBJECT_T>
struct JsonStructBaseDummy {
  static_assert(sizeof(HasJsonStructBase<JS_OBJECT_T>::template test_in_base<JS_OBJECT_T>(nullptr)) ==
                    sizeof(typename HasJsonStructBase<JS_OBJECT_T>::yes),
                "Missing JS_OBJECT JS_OBJECT_EXTERNAL or TypeHandler specialisation\n");
  using TT = decltype(JS_OBJECT_T::template JsonStructBase<JS_OBJECT_T>::js_static_meta_data_info());
  using ST = decltype(JS_OBJECT_T::template JsonStructBase<JS_OBJECT_T>::js_static_meta_super_info());
  static inline constexpr const TT js_static_meta_data_info() {
    return JS_OBJECT_T::template JsonStructBase<JS_OBJECT_T>::js_static_meta_data_info();
  }

  static inline constexpr const ST js_static_meta_super_info() {
    return JS_OBJECT_T::template JsonStructBase<JS_OBJECT_T>::js_static_meta_super_info();
  }
};
}  // namespace Internal

#define JS_INTERNAL_EXPAND(x) x
#define JS_INTERNAL_FIRST_(a, ...) a
#define JS_INTERNAL_SECOND_(a, b, ...) b
#define JS_INTERNAL_FIRST(...) JS_INTERNAL_EXPAND(JS_INTERNAL_FIRST_(__VA_ARGS__))
#define JS_INTERNAL_SECOND(...) JS_INTERNAL_EXPAND(JS_INTERNAL_SECOND_(__VA_ARGS__))
#define JS_INTERNAL_EMPTY()
#define JS_INTERNAL_EVAL(...) JS_INTERNAL_EVAL1024(__VA_ARGS__)
#define JS_INTERNAL_EVAL1024(...) JS_INTERNAL_EVAL512(JS_INTERNAL_EVAL512(__VA_ARGS__))
#define JS_INTERNAL_EVAL512(...) JS_INTERNAL_EVAL256(JS_INTERNAL_EVAL256(__VA_ARGS__))
#define JS_INTERNAL_EVAL256(...) JS_INTERNAL_EVAL128(JS_INTERNAL_EVAL128(__VA_ARGS__))
#define JS_INTERNAL_EVAL128(...) JS_INTERNAL_EVAL64(JS_INTERNAL_EVAL64(__VA_ARGS__))
#define JS_INTERNAL_EVAL64(...) JS_INTERNAL_EVAL32(JS_INTERNAL_EVAL32(__VA_ARGS__))
#define JS_INTERNAL_EVAL32(...) JS_INTERNAL_EVAL16(JS_INTERNAL_EVAL16(__VA_ARGS__))
#define JS_INTERNAL_EVAL16(...) JS_INTERNAL_EVAL8(JS_INTERNAL_EVAL8(__VA_ARGS__))
#define JS_INTERNAL_EVAL8(...) JS_INTERNAL_EVAL4(JS_INTERNAL_EVAL4(__VA_ARGS__))
#define JS_INTERNAL_EVAL4(...) JS_INTERNAL_EVAL2(JS_INTERNAL_EVAL2(__VA_ARGS__))
#define JS_INTERNAL_EVAL2(...) JS_INTERNAL_EVAL1(JS_INTERNAL_EVAL1(__VA_ARGS__))
#define JS_INTERNAL_EVAL1(...) __VA_ARGS__

#define JS_INTERNAL_DEFER1(m) m JS_INTERNAL_EMPTY()
#define JS_INTERNAL_DEFER2(m) m JS_INTERNAL_EMPTY JS_INTERNAL_EMPTY()()

#define JS_INTERNAL_IS_PROBE(...) JS_INTERNAL_SECOND(__VA_ARGS__, 0, 0)
#define JS_INTERNAL_PROBE() ~, 1

#define JS_INTERNAL_CAT(a, b) a##b

#define JS_INTERNAL_NOT(x) JS_INTERNAL_IS_PROBE(JS_INTERNAL_CAT(JS_INTERNAL__NOT_, x))
#define JS_INTERNAL__NOT_0 JS_INTERNAL_PROBE()

#define JS_INTERNAL_BOOL(x) JS_INTERNAL_NOT(JS_INTERNAL_NOT(x))

#define JS_INTERNAL_IF_ELSE(condition) JS_INTERNAL__IF_ELSE(JS_INTERNAL_BOOL(condition))
#define JS_INTERNAL__IF_ELSE(condition) JS_INTERNAL_CAT(JS_INTERNAL__IF_, condition)

#define JS_INTERNAL__IF_1(...) __VA_ARGS__ JS_INTERNAL__IF_1_ELSE
#define JS_INTERNAL__IF_0(...) JS_INTERNAL__IF_0_ELSE

#define JS_INTERNAL__IF_1_ELSE(...)
#define JS_INTERNAL__IF_0_ELSE(...) __VA_ARGS__

#define JS_INTERNAL_HAS_MORE_THAN_ONE_ARGS(...) \
  JS_INTERNAL_BOOL(JS_INTERNAL_SECOND(JS_INTERNAL__END_OF_ARGUMENTS_ __VA_ARGS__, 0, 0)())
#define JS_INTERNAL__END_OF_ARGUMENTS_() 0

#define JS_MEMBER(member) JS::makeMemberInfo(#member, &JS_OBJECT_T::member)
#define JS_MEMBER_ALIASES(member, ...) \
  JS_INTERNAL_EXPAND(JS::makeMemberInfo(#member, &JS_OBJECT_T::member, __VA_ARGS__))
#define JS_MEMBER_WITH_NAME(member, name) JS::makeMemberInfo(name, &JS_OBJECT_T::member)
#define JS_MEMBER_WITH_NAME_AND_ALIASES(member, name, ...) JS::makeMemberInfo(name, &JS_OBJECT_T::member, __VA_ARGS__)

#define JS_SUPER_CLASS(super) JS::makeSuperInfo<super>(#super)

#define JS_SUPER_CLASSES(...) JS::makeTuple(__VA_ARGS__)
#define JS_INTERNAL__MAP_MEMBER() JS_INTERNAL_MAP_MEMBER

#define JS_INTERNAL_MAKE_MEMBERS(...)                                                              \
  JS_INTERNAL_IF_ELSE(JS_INTERNAL_HAS_MORE_THAN_ONE_ARGS(__VA_ARGS__))                             \
  (JS_INTERNAL_EXPAND(JS_INTERNAL_EVAL(JS_INTERNAL_MAP_MEMBER(JS::makeMemberInfo, __VA_ARGS__))))( \
      JS_INTERNAL_MAP_APPLY_MEMBER(JS::makeMemberInfo, __VA_ARGS__))

#define JS_INTERNAL_MAP_APPLY_MEMBER(m, first) m(#first, &JS_OBJECT_T::first)

#define JS_INTERNAL_MAP_MEMBER(m, first, ...)                          \
  JS_INTERNAL_MAP_APPLY_MEMBER(m, first)                               \
  JS_INTERNAL_IF_ELSE(JS_INTERNAL_HAS_MORE_THAN_ONE_ARGS(__VA_ARGS__)) \
  (, JS_INTERNAL_DEFER2(JS_INTERNAL__MAP_MEMBER)()(m, __VA_ARGS__))(, JS_INTERNAL_MAP_APPLY_MEMBER(m, __VA_ARGS__))

#define JS_INTERNAL_MAP_APPLY_SUPER(m, first) m<first>(#first)

#define JS_INTERNAL_MAP_SUPER(m, first, ...)                           \
  JS_INTERNAL_MAP_APPLY_SUPER(m, first)                                \
  JS_INTERNAL_IF_ELSE(JS_INTERNAL_HAS_MORE_THAN_ONE_ARGS(__VA_ARGS__)) \
  (, JS_INTERNAL_DEFER2(JS_INTERNAL__MAP_SUPER)()(m, __VA_ARGS__))(, JS_INTERNAL_MAP_APPLY_SUPER(m, __VA_ARGS__))

#define JS_INTERNAL__MAP_SUPER() JS_INTERNAL_MAP_SUPER

#define JS_INTERNAL_MAKE_SUPER_CLASSES(...)                                                      \
  JS_INTERNAL_IF_ELSE(JS_INTERNAL_HAS_MORE_THAN_ONE_ARGS(__VA_ARGS__))                           \
  (JS_INTERNAL_EXPAND(JS_INTERNAL_EVAL(JS_INTERNAL_MAP_SUPER(JS::makeSuperInfo, __VA_ARGS__))))( \
      JS_INTERNAL_MAP_APPLY_SUPER(JS::makeSuperInfo, __VA_ARGS__))

#define JS_SUPER(...) JS::makeTuple(JS_INTERNAL_EXPAND(JS_INTERNAL_MAKE_SUPER_CLASSES(__VA_ARGS__)))

#define JS_OBJECT_INTERNAL_IMPL(super_list, member_list)                             \
  template <typename JS_OBJECT_T>                                                    \
  struct JsonStructBase {                                                            \
    using TT = decltype(member_list);                                                \
    static inline constexpr const TT js_static_meta_data_info() {                    \
      return member_list;                                                            \
    }                                                                                \
    static inline constexpr const decltype(super_list) js_static_meta_super_info() { \
      return super_list;                                                             \
    }                                                                                \
  }

#define JS_OBJECT_EXTERNAL_INTERNAL_IMPL(Type, super_list, member_list)       \
  namespace JS {                                                              \
  namespace Internal {                                                        \
  template <typename JS_OBJECT_T>                                             \
  struct JsonStructBaseDummy<Type, JS_OBJECT_T> {                             \
    using TT = decltype(member_list);                                         \
    static constexpr const TT js_static_meta_data_info() {                    \
      return member_list;                                                     \
    }                                                                         \
    static constexpr const decltype(super_list) js_static_meta_super_info() { \
      return super_list;                                                      \
    }                                                                         \
  };                                                                          \
  }                                                                           \
  }

#define JS_OBJECT(...) JS_OBJECT_INTERNAL_IMPL(JS::makeTuple(), JS::makeTuple(__VA_ARGS__))
#define JS_OBJECT_WITH_SUPER(super_list, ...) JS_OBJECT_INTERNAL_IMPL(super_list, JS::makeTuple(__VA_ARGS__))

#define JS_OBJECT_EXTERNAL(Type, ...) \
  JS_OBJECT_EXTERNAL_INTERNAL_IMPL(Type, JS::makeTuple(), JS::makeTuple(__VA_ARGS__))
#define JS_OBJECT_EXTERNAL_WITH_SUPER(Type, super_list, ...) \
  JS_OBJECT_EXTERNAL_INTERNAL_IMPL(Type, super_list, JS::makeTuple(__VA_ARGS__))

#define JS_OBJ(...) JS_OBJECT_INTERNAL_IMPL(JS::makeTuple(), JS::makeTuple(JS_INTERNAL_MAKE_MEMBERS(__VA_ARGS__)))
#define JS_OBJ_SUPER(super_list, ...) \
  JS_OBJECT_INTERNAL_IMPL(super_list, JS::makeTuple(JS_INTERNAL_MAKE_MEMBERS(__VA_ARGS__)))

#define JS_OBJ_EXT(Type, ...) \
  JS_OBJECT_EXTERNAL_INTERNAL_IMPL(Type, JS::makeTuple(), JS::makeTuple(JS_INTERNAL_MAKE_MEMBERS(__VA_ARGS__)))
#define JS_OBJ_EXT_SUPER(Type, super_list, ...) \
  JS_OBJECT_EXTERNAL_INTERNAL_IMPL(Type, super_list, JS::makeTuple(JS_INTERNAL_MAKE_MEMBERS(__VA_ARGS__)))

/*!
 * \private
 */
template <typename T, typename U, typename NAMETUPLE>
struct MI {
  NAMETUPLE names;
  T U::*member;
  typedef T type;
};

namespace Internal {
template <typename T, typename U, typename NAMETUPLE>
using MemberInfo = MI<T, U, NAMETUPLE>;

template <typename T>
struct SuperInfo {
  constexpr explicit SuperInfo()
      : name() {
  }
  constexpr explicit SuperInfo(const DataRef &name)
      : name(name) {
  }
  const DataRef name;
  typedef T type;
};
}  // namespace Internal

template <typename T, typename U, size_t NAME_SIZE, typename... Aliases>
constexpr auto makeMemberInfo(const char (&name)[NAME_SIZE], T U::*member, Aliases &... aliases)
    -> MI<T, U, decltype(makeTuple(JS::Internal::makeStringLiteral(name), JS::Internal::makeStringLiteral(aliases)...))> {
  return {makeTuple(JS::Internal::makeStringLiteral(name), JS::Internal::makeStringLiteral(aliases)...), member};
}

template <typename T, size_t NAME_SIZE>
constexpr const Internal::SuperInfo<T> makeSuperInfo(const char (&name)[NAME_SIZE]) {
  return Internal::SuperInfo<T>(DataRef(name));
}

template <typename T, typename Enable = void>
struct TypeHandler {
  static inline Error to(T &to_type, ParseContext &context);
  static inline void from(const T &from_type, Token &token, Serializer &serializer);
};

namespace Internal {
template <size_t STRINGSIZE>
inline bool compareDataRefWithStringLiteral(const StringLiteral<STRINGSIZE> &memberName, const DataRef &jsonName) {
  return jsonName.size == STRINGSIZE && memcmp(memberName.data, jsonName.data, STRINGSIZE) == 0;
}

template <typename NameTuple, size_t index>
struct NameChecker {
  static bool compare(const NameTuple &tuple, const DataRef &name) {
    JS_IF_CONSTEXPR(index != NameTuple::size) {
      auto &stringLiteral = tuple.template get<NameTuple::size - index>();
      if (compareDataRefWithStringLiteral(stringLiteral, name))
        return true;
    }
    return NameChecker<NameTuple, index - 1>::compare(tuple, name);
  }
};
template <typename NameTuple>
struct NameChecker<NameTuple, 0> {
  static bool compare(const NameTuple &tuple, const DataRef &name) {
    JS_UNUSED(tuple);
    JS_UNUSED(name);
    return false;
  }
};

template <typename T, typename MI_T, typename MI_M, typename MI_NC>
inline Error unpackMember(T &to_type, const MemberInfo<MI_T, MI_M, MI_NC> &memberInfo, ParseContext &context,
                          size_t index, bool primary, bool *assigned_members) {
  if (primary) {
    if (compareDataRefWithStringLiteral(memberInfo.names.template get<0>(), context.token.name)) {
      assigned_members[index] = true;
      return TypeHandler<MI_T>::to(to_type.*memberInfo.member, context);
    }
  } else {
    if (NameChecker<MI_NC, MI_NC::size>::compare(memberInfo.names, context.token.name)) {
      assigned_members[index] = true;
      return TypeHandler<MI_T>::to(to_type.*memberInfo.member, context);
    }
  }
  return Error::MissingPropertyMember;
}

template <typename MI_T, typename MI_M, typename MI_NC>
inline Error verifyMember(const MemberInfo<MI_T, MI_M, MI_NC> &memberInfo, size_t index, bool *assigned_members,
                          bool track_missing_members, std::vector<std::string> &missing_members, const char *super_name) {
  if (assigned_members[index])
    return Error::NoError;
  if (IsOptionalType<MI_T>::value)
    return Error::NoError;

  if (track_missing_members) {
    std::string to_push = strlen(super_name) ? std::string(super_name) + "::" : std::string();
    to_push += std::string(memberInfo.names.template get<0>().data, memberInfo.names.template get<0>().size);
    missing_members.push_back(to_push);
  }
  return Error::UnassignedRequiredMember;
}

template <typename T, typename MI_T, typename MI_M, typename MI_NC>
inline void serializeMember(const T &from_type, const MemberInfo<MI_T, MI_M, MI_NC> &memberInfo, Token &token,
                            Serializer &serializer, const char *super_name) {
  JS_UNUSED(super_name);
  token.name.data = memberInfo.names.template get<0>().data;
  token.name.size = memberInfo.names.template get<0>().size;
  token.name_type = Type::Ascii;

  TypeHandler<MI_T>::from(from_type.*memberInfo.member, token, serializer);
}

template <typename T, size_t PAGE, size_t INDEX>
struct SuperClassHandler {
  static Error handleSuperClasses(T &to_type, ParseContext &context, bool primary, bool *assigned_members);
  static Error verifyMembers(bool *assigned_members, bool track_missing_members,
                             std::vector<std::string> &missing_members);
  static constexpr size_t membersInSuperClasses();
  static void serializeMembers(const T &from_type, Token &token, Serializer &serializer);
};

template <typename T, size_t PAGE, size_t SIZE>
struct StartSuperRecursion {
  static Error start(T &to_type, ParseContext &context, bool primary, bool *assigned) {
    return SuperClassHandler<T, PAGE, SIZE - 1>::handleSuperClasses(to_type, context, primary, assigned);
  }

  static Error verifyMembers(bool *assigned_members, bool track_missing_members,
                             std::vector<std::string> &missing_members) {
    return SuperClassHandler<T, PAGE, SIZE - 1>::verifyMembers(assigned_members, track_missing_members,
                                                               missing_members);
  }

  static constexpr size_t membersInSuperClasses() {
    return SuperClassHandler<T, PAGE, SIZE - 1>::membersInSuperClasses();
  }

  static void serializeMembers(const T &from_type, Token &token, Serializer &serializer) {
    return SuperClassHandler<T, PAGE, SIZE - 1>::serializeMembers(from_type, token, serializer);
  }
};

template <typename T, size_t PAGE>
constexpr size_t memberCount() {
  using Members = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_data_info());
  using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
  return Members::size + StartSuperRecursion<T, PAGE + Members::size, SuperMeta::size>::membersInSuperClasses();
}

template <typename T, size_t PAGE>
struct StartSuperRecursion<T, PAGE, 0> {
  static Error start(T &to_type, ParseContext &context, bool primary, bool *assigned) {
    JS_UNUSED(to_type);
    JS_UNUSED(context);
    JS_UNUSED(primary);
    JS_UNUSED(assigned);
    return Error::MissingPropertyMember;
  }

  static Error verifyMembers(bool *assigned_members, bool track_missing_members,
                             std::vector<std::string> &missing_members) {
    JS_UNUSED(assigned_members);
    JS_UNUSED(track_missing_members);
    JS_UNUSED(missing_members);
    return Error::NoError;
  }

  static constexpr size_t membersInSuperClasses() {
    return 0;
  }

  static void serializeMembers(const T &from_type, Token &token, Serializer &serializer) {
    JS_UNUSED(from_type);
    JS_UNUSED(token);
    JS_UNUSED(serializer);
  }
};

template <typename T, typename Members, size_t PAGE, size_t INDEX>
struct MemberChecker {
  inline static Error unpackMembers(T &to_type, const Members &members, ParseContext &context, bool primary,
                                    bool *assigned_members) {
    Error error =
        unpackMember(to_type, members.template get<INDEX>(), context, PAGE + INDEX, primary, assigned_members);
    if (error != Error::MissingPropertyMember)
      return error;

    return MemberChecker<T, Members, PAGE, INDEX - 1>::unpackMembers(to_type, members, context, primary,
                                                                     assigned_members);
  }

  inline static Error verifyMembers(const Members &members, bool *assigned_members, bool track_missing_members,
                                    std::vector<std::string> &missing_members, const char *super_name) {
    Error memberError = verifyMember(members.template get<INDEX>(), PAGE + INDEX, assigned_members,
                                     track_missing_members, missing_members, super_name);
    Error error = MemberChecker<T, Members, PAGE, INDEX - 1>::verifyMembers(
        members, assigned_members, track_missing_members, missing_members, super_name);
    if (memberError != Error::NoError)
      return memberError;
    return error;
  }
  inline static void serializeMembers(const T &from_type, const Members &members, Token &token, Serializer &serializer,
                                      const char *super_name) {
    serializeMember(from_type, members.template get<Members::size - INDEX - 1>(), token, serializer, super_name);
    MemberChecker<T, Members, PAGE, INDEX - 1>::serializeMembers(from_type, members, token, serializer, super_name);
  }
};

template <typename T, typename Members, size_t PAGE>
struct MemberChecker<T, Members, PAGE, 0> {
  inline static Error unpackMembers(T &to_type, const Members &members, ParseContext &context, bool primary,
                                    bool *assigned_members) {
    Error error = unpackMember(to_type, members.template get<0>(), context, PAGE, primary, assigned_members);
    if (error != Error::MissingPropertyMember)
      return error;

    using Super = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
    return StartSuperRecursion<T, PAGE + Members::size, Super::size>::start(to_type, context, primary,
                                                                            assigned_members);
  }

  inline static Error verifyMembers(const Members &members, bool *assigned_members, bool track_missing_members,
                                    std::vector<std::string> &missing_members, const char *super_name) {
    Error memberError = verifyMember(members.template get<0>(), PAGE, assigned_members, track_missing_members,
                                     missing_members, super_name);
    using Super = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
    Error superError = StartSuperRecursion<T, PAGE + Members::size, Super::size>::verifyMembers(
        assigned_members, track_missing_members, missing_members);
    if (memberError != Error::NoError)
      return memberError;
    return superError;
  }

  inline static void serializeMembers(const T &from_type, const Members &members, Token &token, Serializer &serializer,
                                      const char *super_name) {
    serializeMember(from_type, members.template get<Members::size - 1>(), token, serializer, super_name);
    using Super = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
    StartSuperRecursion<T, PAGE + Members::size, Super::size>::serializeMembers(from_type, token, serializer);
  }
};

template <typename T, size_t PAGE, size_t INDEX>
Error SuperClassHandler<T, PAGE, INDEX>::handleSuperClasses(T &to_type, ParseContext &context, bool primary,
                                                            bool *assigned_members) {
  using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
  using Super = typename JS::TypeAt<INDEX, SuperMeta>::type::type;
  using Members = decltype(Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info());
  auto members = Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info();
  Error error = MemberChecker<Super, Members, PAGE, Members::size - 1>::unpackMembers(
      static_cast<Super &>(to_type), members, context, primary, assigned_members);
  if (error != Error::MissingPropertyMember)
    return error;
  return SuperClassHandler<T, PAGE + memberCount<Super, 0>(), INDEX - 1>::handleSuperClasses(to_type, context, primary,
                                                                                             assigned_members);
}

template <typename T, size_t PAGE, size_t INDEX>
Error SuperClassHandler<T, PAGE, INDEX>::verifyMembers(bool *assigned_members, bool track_missing_members,
                                                       std::vector<std::string> &missing_members) {
  using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
  using Super = typename TypeAt<INDEX, SuperMeta>::type::type;
  using Members = decltype(Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info());
  auto members = Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info();
  const char *super_name =
      Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info().template get<INDEX>().name.data;
  Error error = MemberChecker<Super, Members, PAGE, Members::size - 1>::verifyMembers(
      members, assigned_members, track_missing_members, missing_members, super_name);
  Error superError = SuperClassHandler<T, PAGE + memberCount<Super, 0>(), INDEX - 1>::verifyMembers(
      assigned_members, track_missing_members, missing_members);
  if (error != Error::NoError)
    return error;
  return superError;
}

template <typename T, size_t PAGE, size_t INDEX>
size_t constexpr SuperClassHandler<T, PAGE, INDEX>::membersInSuperClasses() {
  using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
  using Super = typename TypeAt<INDEX, SuperMeta>::type::type;
  return memberCount<Super, PAGE>() +
         SuperClassHandler<T, PAGE + memberCount<Super, PAGE>(), INDEX - 1>::membersInSuperClasses();
}

template <typename T, size_t PAGE, size_t INDEX>
void SuperClassHandler<T, PAGE, INDEX>::serializeMembers(const T &from_type, Token &token, Serializer &serializer) {
  using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
  using Super = typename TypeAt<INDEX, SuperMeta>::type::type;
  using Members = decltype(Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info());
  auto members = Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info();
  MemberChecker<Super, Members, PAGE, Members::size - 1>::serializeMembers(from_type, members, token, serializer, "");
  SuperClassHandler<T, PAGE + memberCount<Super, 0>(), INDEX - 1>::serializeMembers(from_type, token, serializer);
}

template <typename T, size_t PAGE>
struct SuperClassHandler<T, PAGE, 0> {
  static Error handleSuperClasses(T &to_type, ParseContext &context, bool primary, bool *assigned_members) {
    using Meta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
    using Super = typename TypeAt<0, Meta>::type::type;
    using Members = decltype(Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info());
    auto members = Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info();
    return MemberChecker<Super, Members, PAGE, Members::size - 1>::unpackMembers(static_cast<Super &>(to_type), members,
                                                                                 context, primary, assigned_members);
  }
  static Error verifyMembers(bool *assigned_members, bool track_missing_members,
                             std::vector<std::string> &missing_members) {
    using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
    using Super = typename TypeAt<0, SuperMeta>::type::type;
    using Members = decltype(Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info());
    auto members = Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info();
    const char *super_name =
        Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info().template get<0>().name.data;
    return MemberChecker<Super, Members, PAGE, Members::size - 1>::verifyMembers(
        members, assigned_members, track_missing_members, missing_members, super_name);
  }
  constexpr static size_t membersInSuperClasses() {
    using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
    using Super = typename TypeAt<0, SuperMeta>::type::type;
    return memberCount<Super, PAGE>();
  }
  static void serializeMembers(const T &from_type, Token &token, Serializer &serializer) {
    using SuperMeta = decltype(Internal::template JsonStructBaseDummy<T, T>::js_static_meta_super_info());
    using Super = typename TypeAt<0, SuperMeta>::type::type;
    using Members = decltype(Internal::template JsonStructBaseDummy<Super, Super>::js_static_meta_data_info());
    auto members = Internal::JsonStructBaseDummy<Super, Super>::js_static_meta_data_info();
    MemberChecker<Super, Members, PAGE, Members::size - 1>::serializeMembers(from_type, members, token, serializer, "");
  }
};

static bool skipArrayOrObject(ParseContext &context) {
  assert(context.error == Error::NoError);
  Type start_type = context.token.value_type;
  Type end_type;
  if (context.token.value_type == Type::ObjectStart) {
    end_type = Type::ObjectEnd;
  } else if (context.token.value_type == Type::ArrayStart) {
    end_type = Type::ArrayEnd;
  } else {
    return false;
  }

  int depth = 1;
  while (depth > 0) {
    context.nextToken();
    if (context.error != Error::NoError) {
      return false;
    }
    if (context.token.value_type == start_type) {
      depth++;
    } else if (context.token.value_type == end_type) {
      depth--;
    }
  }

  return context.token.value_type == end_type && context.error == Error::NoError;
}
}  // namespace Internal

template <typename T>
JS_NODISCARD inline Error ParseContext::parseTo(T &to_type) {
  missing_members.reserve(10);
  unassigned_required_members.reserve(10);
  error = tokenizer.nextToken(token);
  if (error != JS::Error::NoError)
    return error;
  error = TypeHandler<T>::to(to_type, *this);
  if (error != JS::Error::NoError && tokenizer.errorContext().error == JS::Error::NoError) {
    tokenizer.updateErrorContext(error);
  }
  return error;
}

struct SerializerContext {
  SerializerContext(std::string &json_out_p)
      : serializer(), cb_ref(serializer.addRequestBufferCallback([this](Serializer &serializer_p) {
          size_t end = this->json_out.size();
          this->json_out.resize(end * 2);
          serializer_p.setBuffer(&(this->json_out[0]) + end, end);
          this->last_pos = end;
        })),
        json_out(json_out_p),
        last_pos(0) {
    if (json_out.empty())
      json_out.resize(4096);
    serializer.setBuffer(&json_out[0], json_out.size());
  }

  ~SerializerContext() {
    flush();
  }

  template <typename T>
  void serialize(const T &type) {
    JS::Token token;
    JS::TypeHandler<T>::from(type, token, serializer);
    flush();
  }

  void flush() {
    json_out.resize(last_pos + serializer.currentBuffer().used);
  }

  Serializer serializer;
  BufferRequestCBRef cb_ref;
  std::string &json_out;
  size_t last_pos;
};

template <typename T>
JS_NODISCARD std::string serializeStruct(const T &from_type) {
  std::string ret_string;
  SerializerContext serializeContext(ret_string);
  Token token;
  TypeHandler<T>::from(from_type, token, serializeContext.serializer);
  serializeContext.flush();
  return ret_string;
}

template <typename T>
JS_NODISCARD std::string serializeStruct(const T &from_type, const SerializerOptions &options) {
  std::string ret_string;
  SerializerContext serializeContext(ret_string);
  serializeContext.serializer.setOptions(options);
  Token token;
  TypeHandler<T>::from(from_type, token, serializeContext.serializer);
  serializeContext.flush();
  return ret_string;
}

template <>
struct TypeHandler<Error> {
  static inline Error to(Error &to_type, ParseContext &context) {
    (void)to_type;
    (void)context;
    //		if (context.token.value_type == JS::Type::Number) {
    //			int x;
    //			Error error = TypeHandler<int>::to(x, context);
    //			for (int i = 0; i < )
    //		}

    //        size_t level = 1;
    //        Error error = Error::NoError;
    //        while (error == JS::Error::NoError && level) {
    //            error = context.nextToken();
    //            if (context.token.value_type == Type::ObjectStart)
    //                level++;
    //            else if (context.token.value_type == Type::ObjectEnd)
    //                level--;
    //        }

    //        context.tokenizer.copyIncludingValue(context.token, to_type.data);

    return Error::NoError;
  }

  static inline void from(const Error &from_type, Token &token, Serializer &serializer) {
    token.value_type = JS::Type::String;
    if (from_type < JS::Error::UserDefinedErrors) {
      token.value = DataRef(Internal::error_strings[(int)from_type]);
    } else {
      token.value = DataRef("UserDefinedError");
    }
    serializer.write(token);
  }
};

struct CallFunctionExecutionState {
  explicit CallFunctionExecutionState(const std::string &name)
      : name(name), error(Error::NoError) {
  }
  std::string name;
  SilentString context;
  Error error;
  SilentString error_string;
  SilentVector<std::string> missing_members;
  SilentVector<std::string> unassigned_required_members;
  SilentVector<CallFunctionExecutionState> child_states;
  JS_OBJECT(JS_MEMBER(name), JS_MEMBER(context), JS_MEMBER(error), JS_MEMBER(error_string), JS_MEMBER(missing_members),
            JS_MEMBER(unassigned_required_members), JS_MEMBER(child_states));
};

struct CallFunctionContext;

struct CallFunctionErrorContext {
  CallFunctionErrorContext(CallFunctionContext &context)
      : context(context) {
  }

  Error setError(Error error, const std::string &error_string);
  Error setError(const std::string &error_string) {
    return setError(Error::UserDefinedErrors, error_string);
  }
  Error getLatestError() const;

 private:
  CallFunctionContext &context;
};

struct CallFunctionContext {
  CallFunctionContext(ParseContext &parser_context, Serializer &return_serializer)
      : parse_context(parser_context), return_serializer(return_serializer), error_context(*this) {
  }

  virtual ~CallFunctionContext() {
  }
  template <typename T>
  Error callFunctions(T &container);

  ParseContext &parse_context;
  Serializer &return_serializer;
  CallFunctionErrorContext error_context;
  std::vector<CallFunctionExecutionState> execution_list;
  std::string user_context;
  bool allow_missing = false;
  bool stop_execute_on_fail = false;
  void *user_handle = nullptr;

 protected:
  virtual void beforeCallFunctions() {
  }
  virtual void afterCallFunctions() {
  }
};

inline Error CallFunctionErrorContext::setError(Error error, const std::string &errorString) {
  context.parse_context.error = error;
  if (context.execution_list.size()) {
    context.execution_list.back().error = error;
    context.execution_list.back().error_string.data = context.parse_context.tokenizer.makeErrorString();
  }
  context.parse_context.tokenizer.updateErrorContext(error, errorString);
  return error;
}

inline Error CallFunctionErrorContext::getLatestError() const {
  return context.parse_context.error;
}

template <typename T, typename Ret, typename Arg, size_t NAME_COUNT, size_t TAKES_CONTEXT>
struct FunctionInfo {
  typedef Ret (T::*Function)(Arg);
  typedef Ret returnType;
  DataRef name[NAME_COUNT];
  Function function;
};

/// \private
template <typename T, typename Ret, typename Arg, size_t NAME_COUNT>
struct FunctionInfo<T, Ret, Arg, NAME_COUNT, 1> {
  typedef Ret (T::*Function)(Arg, CallFunctionErrorContext &);
  typedef Ret returnType;
  DataRef name[NAME_COUNT];
  Function function;
};

/// \private
template <typename T, typename Ret, typename Arg, size_t NAME_COUNT>
struct FunctionInfo<T, Ret, Arg, NAME_COUNT, 2> {
  typedef Ret (T::*Function)(Arg, CallFunctionContext &);
  typedef Ret returnType;
  DataRef name[NAME_COUNT];
  Function function;
};

/// \private
template <typename T, typename Ret, size_t NAME_COUNT, size_t TAKES_CONTEXT>
struct FunctionInfo<T, Ret, void, NAME_COUNT, TAKES_CONTEXT> {
  typedef Ret (T::*Function)(void);
  typedef Ret returnType;
  DataRef name[NAME_COUNT];
  Function function;
};

/// \private
template <typename T, typename Ret, size_t NAME_COUNT>
struct FunctionInfo<T, Ret, void, NAME_COUNT, 1> {
  typedef Ret (T::*Function)(CallFunctionErrorContext &);
  typedef Ret returnType;
  DataRef name[NAME_COUNT];
  Function function;
};

/// \private
template <typename T, typename Ret, size_t NAME_COUNT>
struct FunctionInfo<T, Ret, void, NAME_COUNT, 2> {
  typedef Ret (T::*Function)(CallFunctionContext &);
  typedef Ret returnType;
  DataRef name[NAME_COUNT];
  Function function;
};

/// \private
template <typename T, typename Ret, typename Arg, size_t NAME_SIZE, typename... Aliases>
constexpr FunctionInfo<T, Ret, Arg, sizeof...(Aliases) + 1, 0> makeFunctionInfo(const char (&name)[NAME_SIZE],
                                                                                Ret (T::*function)(Arg),
                                                                                Aliases... aliases) {
  return {{DataRef(name), DataRef(aliases)...}, function};
}

/// \private
template <typename T, typename Ret, typename Arg, size_t NAME_SIZE, typename... Aliases>
constexpr FunctionInfo<T, Ret, Arg, sizeof...(Aliases) + 1, 1> makeFunctionInfo(
    const char (&name)[NAME_SIZE], Ret (T::*function)(Arg, CallFunctionErrorContext &), Aliases... aliases) {
  return {{DataRef(name), DataRef(aliases)...}, function};
}

/// \private
template <typename T, typename Ret, typename Arg, size_t NAME_SIZE, typename... Aliases>
constexpr FunctionInfo<T, Ret, Arg, sizeof...(Aliases) + 1, 2> makeFunctionInfo(
    const char (&name)[NAME_SIZE], Ret (T::*function)(Arg, CallFunctionContext &), Aliases... aliases) {
  return {{DataRef(name), DataRef(aliases)...}, function};
}

/// \private
template <typename T, typename Ret, size_t NAME_SIZE, typename... Aliases>
constexpr FunctionInfo<T, Ret, void, sizeof...(Aliases) + 1, 0> makeFunctionInfo(const char (&name)[NAME_SIZE],
                                                                                 Ret (T::*function)(void),
                                                                                 Aliases... aliases) {
  return {{DataRef(name), DataRef(aliases)...}, function};
}

/// \private
template <typename T, typename Ret, size_t NAME_SIZE, typename... Aliases>
constexpr FunctionInfo<T, Ret, void, sizeof...(Aliases) + 1, 1> makeFunctionInfo(
    const char (&name)[NAME_SIZE], Ret (T::*function)(CallFunctionErrorContext &), Aliases... aliases) {
  return {{DataRef(name), DataRef(aliases)...}, function};
}

/// \private
template <typename T, typename Ret, size_t NAME_SIZE, typename... Aliases>
constexpr FunctionInfo<T, Ret, void, sizeof...(Aliases) + 1, 2> makeFunctionInfo(
    const char (&name)[NAME_SIZE], Ret (T::*function)(CallFunctionContext &), Aliases... aliases) {
  return {{DataRef(name), DataRef(aliases)...}, function};
}

namespace Internal {
template <typename T>
struct HasJsonStructFunctionContainer {
  typedef char yes[1];
  typedef char no[2];

  template <typename C>
  static constexpr yes &test_in_base(typename C::template JsonStructFunctionContainer<C> *);

  template <typename>
  static constexpr no &test_in_base(...);
};

template <typename JS_BASE_CONTAINER_STRUCT_T, typename JS_CONTAINER_STRUCT_T>
struct JsonStructFunctionContainerDummy {
  using TT = decltype(JS_CONTAINER_STRUCT_T::template JsonStructFunctionContainer<
                      JS_CONTAINER_STRUCT_T>::js_static_meta_functions_info());
  using ST = decltype(
      JS_CONTAINER_STRUCT_T::template JsonStructFunctionContainer<JS_CONTAINER_STRUCT_T>::js_static_meta_super_info());
  static const TT &js_static_meta_functions_info() {
    return JS_CONTAINER_STRUCT_T::template JsonStructFunctionContainer<
        JS_CONTAINER_STRUCT_T>::js_static_meta_functions_info();
  }

  static const ST js_static_meta_super_info() {
    return JS_CONTAINER_STRUCT_T::template JsonStructFunctionContainer<
        JS_CONTAINER_STRUCT_T>::js_static_meta_super_info();
  }
};

}  // namespace Internal

#define JS_FUNCTION(name) JS::makeFunctionInfo(#name, &JS_CONTAINER_STRUCT_T::name)
#define JS_FUNCTION_ALIASES(name, ...) JS::makeFunctionInfo(#name, &JS_CONTAINER_STRUCT_T::name, __VA_ARGS__)
#define JS_FUNCTION_WITH_NAME(member, name) JS::makeFunctionInfo(name, &JS_CONTAINER_STRUCT_T::member)
#define JS_FUNCTION_WITH_NAME_ALIASES(member, name, ...) \
  JS::makeFunctionInfo(name, &JS_CONTAINER_STRUCT_T::member, __VA_ARGS__)

#define JS_INTERNAL_MAP_APPLY_FUNCTION(m, first) m(#first, &JS_CONTAINER_STRUCT_T::first)

#define JS_INTERNAL_MAP_FUNCTION(m, first, ...)                        \
  JS_INTERNAL_MAP_APPLY_FUNCTION(m, first)                             \
  JS_INTERNAL_IF_ELSE(JS_INTERNAL_HAS_MORE_THAN_ONE_ARGS(__VA_ARGS__)) \
  (, JS_INTERNAL_DEFER2(JS_INTERNAL__MAP_FUNCTION)()(m, __VA_ARGS__))(, JS_INTERNAL_MAP_APPLY_FUNCTION(m, __VA_ARGS__))

#define JS_INTERNAL__MAP_FUNCTION() JS_INTERNAL_MAP_FUNCTION

#define JS_INTERNAL_MAKE_FUNCTIONS(...)                                                                \
  JS_INTERNAL_IF_ELSE(JS_INTERNAL_HAS_MORE_THAN_ONE_ARGS(__VA_ARGS__))                                 \
  (JS_INTERNAL_EXPAND(JS_INTERNAL_EVAL(JS_INTERNAL_MAP_FUNCTION(JS::makeFunctionInfo, __VA_ARGS__))))( \
      JS_INTERNAL_MAP_APPLY_FUNCTION(JS::makeFunctionInfo, __VA_ARGS__))

#define JS_FUNCTION_CONTAINER_INTERNAL_IMPL(super_list, function_list) \
  template <typename JS_CONTAINER_STRUCT_T>                            \
  struct JsonStructFunctionContainer {                                 \
    using TT = decltype(function_list);                                \
    static const TT &js_static_meta_functions_info() {                 \
      static auto ret = function_list;                                 \
      return ret;                                                      \
    }                                                                  \
    static const decltype(super_list) js_static_meta_super_info() {    \
      return super_list;                                               \
    }                                                                  \
  }

#define JS_FUNCTION_CONTAINER_EXTERNAL_INTERNAL_IMPL(Type, super_list, function_list) \
  namespace JS {                                                                      \
  namespace Internal {                                                                \
  template <typename JS_CONTAINER_STRUCT_T>                                           \
  struct JsonStructFunctionContainerDummy<Type, JS_CONTAINER_STRUCT_T> {              \
    using TT = decltype(function_list);                                               \
    static const TT &js_static_meta_functions_info() {                                \
      static auto ret = function_list;                                                \
      return ret;                                                                     \
    }                                                                                 \
    static const decltype(super_list) js_static_meta_super_info() {                   \
      return super_list;                                                              \
    }                                                                                 \
  };                                                                                  \
  }                                                                                   \
  }

#define JS_FUNC_OBJ(...) \
  JS_FUNCTION_CONTAINER_INTERNAL_IMPL(JS::makeTuple(), JS::makeTuple(JS_INTERNAL_MAKE_FUNCTIONS(__VA_ARGS__)))
#define JS_FUNCTION_CONTAINER(...) JS_FUNCTION_CONTAINER_INTERNAL_IMPL(JS::makeTuple(), JS::makeTuple(__VA_ARGS__))
#define JS_FUNC_OBJ_SUPER(super_list, ...) \
  JS_FUNCTION_CONTAINER_INTERNAL_IMPL(super_list, JS::makeTuple(JS_INTERNAL_MAKE_FUNCTIONS(__VA_ARGS__)))
#define JS_FUNCTION_CONTAINER_WITH_SUPER(super_list, ...) \
  JS_FUNCTION_CONTAINER_INTERNAL_IMPL(super_list, JS::makeTuple(__VA_ARGS__))
#define JS_FUNCTION_CONTAINER_WITH_SUPER_WITHOUT_MEMBERS(super_list) \
  JS_FUNCTION_CONTAINER_INTERNAL_IMPL(super_list, JS::makeTuple())

#define JS_FUNC_OBJ_EXTERNAL(Type, ...)                               \
  JS_FUNCTION_CONTAINER_EXTERNAL_INTERNAL_IMPL(Type, JS::makeTuple(), \
                                               JS::makeTuple(JS_INTERNAL_MAKE_FUNCTIONS(__VA_ARGS__)))
#define JS_FUNCTION_CONTAINER_EXTERNAL(Type, ...) \
  JS_FUNCTION_CONTAINER_EXTERNAL_INTERNAL_IMPL(Type, JS::makeTuple(), JS::makeTuple(__VA_ARGS__))
#define JS_FUNC_OBJ_EXTERNAL_SUPER(Type, super_list, ...) \
  JS_FUNCTION_CONTAINER_EXTERNAL_INTERNAL_IMPL(Type, super_list, JS::makeTuple(JS_INTERNAL_MAKE_FUNCTIONS(__VA_ARGS__)))
#define JS_FUNCTION_CONTAINER_EXTERNAL_WITH_SUPER(Type, super_list, ...) \
  JS_FUNCTION_CONTAINER_EXTERNAL_INTERNAL_IMPL(Type, super_list, JS::makeTuple(__VA_ARGS__))

#define JS_FUNCTION_CONTAINER_EXTERNAL_WITH_SUPER_WITHOUT_MEMBERS(Type, super_list) \
  JS_FUNCTION_CONTAINER_EXTERNAL_INTERNAL_IMPL(Type, super_list, JS::makeTuple())

#if !defined(__clang__) && defined(__GNUC__) && __GNUC__ == 11
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Warray-bounds"
#endif

namespace Internal {
template <typename T, typename U, typename Ret, typename Arg, size_t NAME_COUNT, size_t TAKES_CONTEXT>
struct FunctionCaller {
  static Error callFunctionAndSerializeReturn(T &container,
                                              FunctionInfo<U, Ret, Arg, NAME_COUNT, TAKES_CONTEXT> &functionInfo,
                                              CallFunctionContext &context) {
    typedef typename std::remove_reference<Arg>::type NonRefArg;
    typedef typename std::remove_cv<NonRefArg>::type PureArg;
    PureArg arg;
    context.parse_context.error = TypeHandler<PureArg>::to(arg, context.parse_context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    Token token;
    TypeHandler<Ret>::from((container.*functionInfo.function)(arg), token, context.return_serializer);
    return Error::NoError;
  }
};

template <typename T, typename U, typename Ret, typename Arg, size_t NAME_COUNT>
struct FunctionCaller<T, U, Ret, Arg, NAME_COUNT, 1> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, Ret, Arg, NAME_COUNT, 1> &functionInfo,
                                              CallFunctionContext &context) {
    typedef typename std::remove_reference<Arg>::type NonRefArg;
    typedef typename std::remove_cv<NonRefArg>::type PureArg;
    PureArg arg;
    context.parse_context.error = TypeHandler<PureArg>::to(arg, context.parse_context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    Token token;
    Ret ret = (container.*functionInfo.function)(arg, context.error_context);
    if (context.execution_list.back().error == Error::NoError)
      TypeHandler<Ret>::from(ret, token, context.return_serializer);
    return context.execution_list.back().error;
  }
};

template <typename T, typename U, typename Ret, typename Arg, size_t NAME_COUNT>
struct FunctionCaller<T, U, Ret, Arg, NAME_COUNT, 2> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, Ret, Arg, NAME_COUNT, 2> &functionInfo,
                                              CallFunctionContext &context) {
    typedef typename std::remove_reference<Arg>::type NonRefArg;
    typedef typename std::remove_cv<NonRefArg>::type PureArg;
    PureArg arg;
    context.parse_context.error = TypeHandler<PureArg>::to(arg, context.parse_context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    Token token;
    Ret ret = (container.*functionInfo.function)(arg, context);
    if (context.execution_list.back().error == Error::NoError)
      TypeHandler<Ret>::from(ret, token, context.return_serializer);
    return context.execution_list.back().error;
  }
};

template <typename T, typename U, typename Arg, size_t NAME_COUNT, size_t TAKES_CONTEXT>
struct FunctionCaller<T, U, void, Arg, NAME_COUNT, TAKES_CONTEXT> {
  static Error callFunctionAndSerializeReturn(T &container,
                                              FunctionInfo<U, void, Arg, NAME_COUNT, TAKES_CONTEXT> &functionInfo,
                                              CallFunctionContext &context) {
    typedef typename std::remove_reference<Arg>::type NonRefArg;
    typedef typename std::remove_cv<NonRefArg>::type PureArg;
    PureArg arg;
    context.parse_context.error = TypeHandler<PureArg>::to(arg, context.parse_context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    (container.*functionInfo.function)(arg);
    return Error::NoError;
  }
};

template <typename T, typename U, typename Arg, size_t NAME_COUNT>
struct FunctionCaller<T, U, void, Arg, NAME_COUNT, 1> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, void, Arg, NAME_COUNT, 1> &functionInfo,
                                              CallFunctionContext &context) {
    typedef typename std::remove_reference<Arg>::type NonRefArg;
    typedef typename std::remove_cv<NonRefArg>::type PureArg;
    PureArg arg;
    context.parse_context.error = TypeHandler<PureArg>::to(arg, context.parse_context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    (container.*functionInfo.function)(arg, context.error_context);
    return context.execution_list.back().error;
  }
};

template <typename T, typename U, typename Arg, size_t NAME_COUNT>
struct FunctionCaller<T, U, void, Arg, NAME_COUNT, 2> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, void, Arg, NAME_COUNT, 2> &functionInfo,
                                              CallFunctionContext &context) {
    typedef typename std::remove_reference<Arg>::type NonRefArg;
    typedef typename std::remove_cv<NonRefArg>::type PureArg;
    PureArg arg;
    context.parse_context.error = TypeHandler<PureArg>::to(arg, context.parse_context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    (container.*functionInfo.function)(arg, context);
    return context.execution_list.back().error;
  }
};

static inline void checkValidVoidParameter(CallFunctionContext &context) {
  if (context.parse_context.token.value_type != Type::Null &&
      context.parse_context.token.value_type != Type::ArrayStart &&
      context.parse_context.token.value_type != Type::ObjectStart &&
      context.parse_context.token.value_type != Type::Bool) {
    // what to do
    fprintf(stderr, "Passing data arguments to a void function\n");
  }
  skipArrayOrObject(context.parse_context);
}

template <typename T, typename U, typename Ret, size_t NAME_COUNT, size_t TAKES_CONTEXT>
struct FunctionCaller<T, U, Ret, void, NAME_COUNT, TAKES_CONTEXT> {
  static Error callFunctionAndSerializeReturn(T &container,
                                              FunctionInfo<U, Ret, void, NAME_COUNT, TAKES_CONTEXT> &functionInfo,
                                              CallFunctionContext &context) {
    checkValidVoidParameter(context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;
    Token token;
    TypeHandler<Ret>::from((container.*functionInfo.function)(), token, context.return_serializer);
    return Error::NoError;
  }
};

template <typename T, typename U, typename Ret, size_t NAME_COUNT>
struct FunctionCaller<T, U, Ret, void, NAME_COUNT, 1> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, Ret, void, NAME_COUNT, 1> &functionInfo,
                                              CallFunctionContext &context) {
    checkValidVoidParameter(context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    Token token;
    Ret ret = (container.*functionInfo.function)(context.error_context);
    if (context.execution_list.back().error == Error::NoError)
      TypeHandler<Ret>::from(ret, token, context.return_serializer);
    return context.execution_list.back().error;
  }
};

template <typename T, typename U, typename Ret, size_t NAME_COUNT>
struct FunctionCaller<T, U, Ret, void, NAME_COUNT, 2> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, Ret, void, NAME_COUNT, 2> &functionInfo,
                                              CallFunctionContext &context) {
    checkValidVoidParameter(context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    Token token;
    Ret ret = (container.*functionInfo.function)(context);
    if (context.execution_list.back().error == Error::NoError)
      TypeHandler<Ret>::from(ret, token, context.return_serializer);
    return context.execution_list.back().error;
  }
};

template <typename T, typename U, size_t NAME_COUNT, size_t TAKES_CONTEXT>
struct FunctionCaller<T, U, void, void, NAME_COUNT, TAKES_CONTEXT> {
  static Error callFunctionAndSerializeReturn(T &container,
                                              FunctionInfo<U, void, void, NAME_COUNT, TAKES_CONTEXT> &functionInfo,
                                              CallFunctionContext &context) {
    checkValidVoidParameter(context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    (container.*functionInfo.function)();
    return Error::NoError;
  }
};

template <typename T, typename U, size_t NAME_COUNT>
struct FunctionCaller<T, U, void, void, NAME_COUNT, 1> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, void, void, NAME_COUNT, 1> &functionInfo,
                                              CallFunctionContext &context) {
    checkValidVoidParameter(context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    (container.*functionInfo.function)(context.error_context);
    return context.execution_list.back().error;
  }
};

template <typename T, typename U, size_t NAME_COUNT>
struct FunctionCaller<T, U, void, void, NAME_COUNT, 2> {
  static Error callFunctionAndSerializeReturn(T &container, FunctionInfo<U, void, void, NAME_COUNT, 2> &functionInfo,
                                              CallFunctionContext &context) {
    checkValidVoidParameter(context);
    if (context.parse_context.error != Error::NoError)
      return context.parse_context.error;

    (container.*functionInfo.function)(context);
    return context.execution_list.back().error;
  }
};
}  // namespace Internal

#if !defined(__clang__) && defined(__GNUC__) && __GNUC__ == 11
  #pragma GCC diagnostic pop
#endif

template <typename T, typename U, typename Ret, typename Arg, size_t NAME_COUNT, size_t TAKES_CONTEXT>
Error matchAndCallFunction(T &container, CallFunctionContext &context,
                           FunctionInfo<U, Ret, Arg, NAME_COUNT, TAKES_CONTEXT> &functionInfo, bool primary) {
  if (primary && context.parse_context.token.name.size == functionInfo.name[0].size &&
      memcmp(functionInfo.name[0].data, context.parse_context.token.name.data, functionInfo.name[0].size) == 0) {
    return Internal::FunctionCaller<T, U, Ret, Arg, NAME_COUNT, TAKES_CONTEXT>::callFunctionAndSerializeReturn(
        container, functionInfo, context);
  } else if (!primary) {
    for (size_t i = 1; i < NAME_COUNT; i++) {
      if (context.parse_context.token.name.size == functionInfo.name[i].size &&
          memcmp(functionInfo.name[i].data, context.parse_context.token.name.data, functionInfo.name[i].size) == 0) {
        return Internal::FunctionCaller<T, U, Ret, Arg, NAME_COUNT, TAKES_CONTEXT>::callFunctionAndSerializeReturn(
            container, functionInfo, context);
      }
    }
  }
  return Error::MissingFunction;
}

namespace Internal {
template <typename T, size_t INDEX>
struct FunctionalSuperRecursion {
  static Error callFunction(T &container, CallFunctionContext &context, bool primary);
};

template <typename T, size_t SIZE>
struct StartFunctionalSuperRecursion {
  static Error callFunction(T &container, CallFunctionContext &context, bool primary) {
    return FunctionalSuperRecursion<T, SIZE - 1>::callFunction(container, context, primary);
  }
};
template <typename T>
struct StartFunctionalSuperRecursion<T, 0> {
  static Error callFunction(T &container, CallFunctionContext &context, bool primary) {
    JS_UNUSED(container);
    JS_UNUSED(context);
    JS_UNUSED(primary);
    return Error::MissingFunction;
  }
};

template <typename T, typename Functions, size_t INDEX>
struct FunctionObjectTraverser {
  static Error call(T &container, CallFunctionContext &context, Functions &functions, bool primary) {
    auto function = functions.template get<INDEX>();
    Error error = matchAndCallFunction(container, context, function, primary);
    if (error == Error::NoError)
      return Error::NoError;
    if (error != Error::MissingFunction)
      return context.parse_context.error;
    return FunctionObjectTraverser<T, Functions, INDEX - 1>::call(container, context, functions, primary);
  }
};

template <typename T, typename Functions>
struct FunctionObjectTraverser<T, Functions, 0> {
  static Error call(T &container, CallFunctionContext &context, Functions &functions, bool primary) {
    auto function = functions.template get<0>();
    Error error = matchAndCallFunction(container, context, function, primary);
    if (error == Error::NoError)
      return Error::NoError;
    if (error != Error::MissingFunction)
      return error;
    using SuperMeta = decltype(Internal::template JsonStructFunctionContainerDummy<T, T>::js_static_meta_super_info());
    return StartFunctionalSuperRecursion<T, SuperMeta::size>::callFunction(container, context, primary);
  }
};

template <typename T, typename Functions>
struct FunctionObjectTraverser<T, Functions, size_t(-1)> {
  static Error call(T &container, CallFunctionContext &context, Functions &, bool primary) {
    using SuperMeta = decltype(Internal::template JsonStructFunctionContainerDummy<T, T>::js_static_meta_super_info());
    return StartFunctionalSuperRecursion<T, SuperMeta::size>::callFunction(container, context, primary);
  }
};

static inline void add_error(CallFunctionExecutionState &executionState, ParseContext &context) {
  executionState.error = context.error;
  if (context.error != Error::NoError) {
    if (context.tokenizer.errorContext().custom_message.empty())
      context.tokenizer.updateErrorContext(context.error);
    executionState.error_string.data = context.tokenizer.makeErrorString();
  }
  if (context.missing_members.size())
    std::swap(executionState.missing_members.data, context.missing_members);
  if (context.unassigned_required_members.size())
    std::swap(executionState.unassigned_required_members.data, context.unassigned_required_members);
}
}  // namespace Internal

namespace Internal {
typedef void (CallFunctionContext::*AfterCallFunction)();

struct RAICallFunctionOnExit {
  RAICallFunctionOnExit(CallFunctionContext &context, AfterCallFunction after)
      : context(context), after(after) {
  }
  ~RAICallFunctionOnExit() {
    (context.*after)();
  }
  CallFunctionContext &context;
  AfterCallFunction after;
};
}  // namespace Internal

namespace Internal {
struct ArrayEndWriter {
  ArrayEndWriter(Serializer &serializer, Token &token)
      : serializer(serializer), token(token) {
  }

  ~ArrayEndWriter() {
    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }

  Serializer &serializer;
  Token &token;
};
}  // namespace Internal

template <typename T>
inline Error CallFunctionContext::callFunctions(T &container) {
  beforeCallFunctions();
  Internal::RAICallFunctionOnExit callOnExit(*this, &CallFunctionContext::afterCallFunctions);
  JS::Error error = parse_context.nextToken();
  if (error != JS::Error::NoError)
    return error;
  if (parse_context.token.value_type != JS::Type::ObjectStart) {
    return error_context.setError(Error::ExpectedObjectStart, "Can only call functions on objects with members");
  }
  error = parse_context.nextToken();
  if (error != JS::Error::NoError)
    return error;
  Token token;
  token.value_type = Type::ArrayStart;
  token.value = DataRef("[");
  Internal::ArrayEndWriter endWriter(return_serializer, token);
  return_serializer.write(token);
  auto &functions = Internal::JsonStructFunctionContainerDummy<T, T>::js_static_meta_functions_info();
  using FunctionsType = typename std::remove_reference<decltype(functions)>::type;
  while (parse_context.token.value_type != JS::Type::ObjectEnd) {
    parse_context.tokenizer.pushScope(parse_context.token.value_type);
    execution_list.push_back(
        CallFunctionExecutionState(std::string(parse_context.token.name.data, parse_context.token.name.size)));
    execution_list.back().context.data = user_context;
    error = Internal::FunctionObjectTraverser<T, FunctionsType, FunctionsType::size - 1>::call(container, *this,
                                                                                               functions, true);
    if (error == Error::MissingFunction)
      error = Internal::FunctionObjectTraverser<T, FunctionsType, FunctionsType::size - 1>::call(container, *this,
                                                                                                 functions, false);
    if (error != Error::NoError) {
      assert(error == parse_context.error || parse_context.error == Error::NoError);
      parse_context.error = error;
    }
    Internal::add_error(execution_list.back(), parse_context);
    parse_context.tokenizer.goToEndOfScope(parse_context.token);
    parse_context.tokenizer.popScope();
    if (error == Error::MissingFunction && allow_missing)
      error = Error::NoError;
    if (stop_execute_on_fail && error != Error::NoError)
      return error;

    error = parse_context.nextToken();
    if (error != JS::Error::NoError)
      return error;
  }

  return Error::NoError;
}

struct DefaultCallFunctionContext : public CallFunctionContext {
  DefaultCallFunctionContext(std::string &json_out)
      : CallFunctionContext(p_context, s_context.serializer), s_context(json_out) {
  }

  DefaultCallFunctionContext(const char *data, size_t size, std::string &json_out)
      : CallFunctionContext(p_context, s_context.serializer), p_context(data, size), s_context(json_out) {
  }

  template <size_t SIZE>
  DefaultCallFunctionContext(const char (&data)[SIZE], std::string &json_out)
      : CallFunctionContext(p_context, s_context.serializer), p_context(data), s_context(json_out) {
  }

  ParseContext p_context;
  SerializerContext s_context;

 protected:
  void afterCallFunctions() {
    s_context.flush();
  }
};
namespace Internal {
template <typename T, size_t INDEX>
Error FunctionalSuperRecursion<T, INDEX>::callFunction(T &container, CallFunctionContext &context, bool primary) {
  using SuperMeta = decltype(Internal::template JsonStructFunctionContainerDummy<T, T>::js_static_meta_super_info());
  using Super = typename TypeAt<INDEX, SuperMeta>::type::type;
  auto &functions = Internal::template JsonStructFunctionContainerDummy<Super, Super>::js_static_meta_functions_info();
  using FunctionsType = typename std::remove_reference<decltype(functions)>::type;
  Error error = FunctionObjectTraverser<Super, FunctionsType, FunctionsType::size - 1>::call(container, context,
                                                                                             functions, primary);
  if (error != Error::MissingFunction)
    return error;

  return FunctionalSuperRecursion<T, INDEX - 1>::callFunction(container, context, primary);
}

template <typename T>
struct FunctionalSuperRecursion<T, 0> {
  static Error callFunction(T &container, CallFunctionContext &context, bool primary) {
    using SuperMeta = decltype(Internal::template JsonStructFunctionContainerDummy<T, T>::js_static_meta_super_info());
    using Super = typename TypeAt<0, SuperMeta>::type::type;
    auto &functions =
        Internal::template JsonStructFunctionContainerDummy<Super, Super>::js_static_meta_functions_info();
    using FunctionsType = typename std::remove_reference<decltype(functions)>::type;
    return FunctionObjectTraverser<Super, FunctionsType, FunctionsType::size - 1>::call(container, context, functions,
                                                                                        primary);
  }
};
}  // namespace Internal
namespace Internal {
enum class ParseEnumStringState {
  FindingNameStart,
  FindingNameEnd,
  FindingSeperator
};
template <size_t N>
void populateEnumNames(std::vector<DataRef> &names, const char (&data)[N]) {
  size_t name_starts_at = 0;
  ParseEnumStringState state = ParseEnumStringState::FindingNameStart;
  for (size_t i = 0; i < N; i++) {
    char c = data[i];
    assert(c != '=');
    switch (state) {
      case ParseEnumStringState::FindingNameStart:
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
          name_starts_at = i;
          state = ParseEnumStringState::FindingNameEnd;
        }
        break;
      case ParseEnumStringState::FindingNameEnd:
        if (c == '\0' || c == '\t' || c == '\n' || c == '\r' || c == ' ' || c == ',') {
          names.push_back(DataRef(data + name_starts_at, i - name_starts_at));
          state = c == ',' ? ParseEnumStringState::FindingNameStart : ParseEnumStringState::FindingSeperator;
        }
        break;
      case ParseEnumStringState::FindingSeperator:
        if (c == ',')
          state = ParseEnumStringState::FindingNameStart;
        break;
    }
  }
}
}  // namespace Internal
}  // namespace JS

#define JS_ENUM(name, ...)                                      \
  enum class name {                                             \
    __VA_ARGS__                                                 \
  };                                                            \
  struct js_##name##_string_struct {                            \
    template <size_t N>                                         \
    explicit js_##name##_string_struct(const char (&data)[N]) { \
      JS::Internal::populateEnumNames(_strings, data);          \
    }                                                           \
    std::vector<JS::DataRef> _strings;                          \
                                                                \
    static const std::vector<JS::DataRef> &strings() {          \
      static js_##name##_string_struct ret(#__VA_ARGS__);       \
      return ret._strings;                                      \
    }                                                           \
  };

#define JS_ENUM_DECLARE_STRING_PARSER(name)                                                              \
  namespace JS {                                                                                         \
  template <>                                                                                            \
  struct TypeHandler<name> {                                                                             \
    static inline Error to(name &to_type, ParseContext &context) {                                       \
      return Internal::EnumHandler<name, js_##name##_string_struct>::to(to_type, context);               \
    }                                                                                                    \
    static inline void from(const name &from_type, Token &token, Serializer &serializer) {               \
      return Internal::EnumHandler<name, js_##name##_string_struct>::from(from_type, token, serializer); \
    }                                                                                                    \
  };                                                                                                     \
  }

#define JS_ENUM_NAMESPACE_DECLARE_STRING_PARSER(ns, name)                                                        \
  namespace JS {                                                                                                 \
  template <>                                                                                                    \
  struct TypeHandler<ns::name> {                                                                                 \
    static inline Error to(ns::name &to_type, ParseContext &context) {                                           \
      return Internal::EnumHandler<ns::name, ns::js_##name##_string_struct>::to(to_type, context);               \
    }                                                                                                            \
    static inline void from(const ns::name &from_type, Token &token, Serializer &serializer) {                   \
      return Internal::EnumHandler<ns::name, ns::js_##name##_string_struct>::from(from_type, token, serializer); \
    }                                                                                                            \
  };                                                                                                             \
  }

#define JS_ENUM_DECLARE_VALUE_PARSER(name)                                                 \
  namespace JS {                                                                           \
  template <>                                                                              \
  struct TypeHandler<name> {                                                               \
    typedef std::underlying_type<name>::type utype;                                        \
    static inline Error to(name &to_type, ParseContext &context) {                         \
      utype to_value;                                                                      \
      JS::Error result = TypeHandler<utype>::to(to_value, context);                        \
      if (result == JS::Error::NoError)                                                    \
        to_type = static_cast<name>(to_value);                                             \
      return result;                                                                       \
    }                                                                                      \
    static inline void from(const name &from_type, Token &token, Serializer &serializer) { \
      const utype from_value = static_cast<utype>(from_type);                              \
      TypeHandler<utype>::from(from_value, token, serializer);                             \
    }                                                                                      \
  };                                                                                       \
  }

#define JS_ENUM_NAMESPACE_DECLARE_VALUE_PARSER(ns, name)                                       \
  namespace JS {                                                                               \
  template <>                                                                                  \
  struct TypeHandler<ns::name> {                                                               \
    typedef std::underlying_type<ns::name>::type utype;                                        \
    static inline Error to(ns::name &to_type, ParseContext &context) {                         \
      utype to_value;                                                                          \
      JS::Error result = TypeHandler<utype>::to(to_value, context);                            \
      if (result == JS::Error::NoError)                                                        \
        to_type = static_cast<ns::name>(to_value);                                             \
      return result;                                                                           \
    }                                                                                          \
    static inline void from(const ns::name &from_type, Token &token, Serializer &serializer) { \
      const utype from_value = static_cast<utype>(from_type);                                  \
      TypeHandler<utype>::from(from_value, token, serializer);                                 \
    }                                                                                          \
  };                                                                                           \
  }

namespace JS {
template <typename T, typename Enable>
inline Error TypeHandler<T, Enable>::to(T &to_type, ParseContext &context) {
  if (context.token.value_type != JS::Type::ObjectStart)
    return Error::ExpectedObjectStart;
  Error error = context.tokenizer.nextToken(context.token);
  if (error != JS::Error::NoError)
    return error;
  auto members = Internal::JsonStructBaseDummy<T, T>::js_static_meta_data_info();
  using MembersType = decltype(members);
  bool assigned_members[Internal::memberCount<T, 0>()];
  memset(assigned_members, 0, sizeof(assigned_members));
  while (context.token.value_type != JS::Type::ObjectEnd)

  {
    DataRef token_name = context.token.name;
    error = Internal::MemberChecker<T, MembersType, 0, MembersType::size - 1>::unpackMembers(to_type, members, context,
                                                                                             true, assigned_members);
    if (error == Error::MissingPropertyMember)
      error = Internal::MemberChecker<T, MembersType, 0, MembersType::size - 1>::unpackMembers(
          to_type, members, context, false, assigned_members);
    if (error == Error::MissingPropertyMember) {
      if (context.track_member_assignement_state)
        context.missing_members.emplace_back(token_name.data, token_name.data + token_name.size);
      if (context.allow_missing_members) {
        Internal::skipArrayOrObject(context);
        if (context.error != Error::NoError)
          return context.error;
      } else {
        return error;
      }
    } else if (error != Error::NoError) {
      return error;
    }
    context.nextToken();
    if (context.error != Error::NoError)
      return context.error;
  }
  std::vector<std::string> unassigned_required_members;
  error = Internal::MemberChecker<T, MembersType, 0, MembersType::size - 1>::verifyMembers(
      members, assigned_members, context.track_member_assignement_state, unassigned_required_members, "");
  if (error == Error::UnassignedRequiredMember) {
    if (context.track_member_assignement_state)
      context.unassigned_required_members.insert(context.unassigned_required_members.end(),
                                                 unassigned_required_members.begin(),
                                                 unassigned_required_members.end());
    if (context.allow_unasigned_required_members)
      error = Error::NoError;
  }
  return error;
}

template <typename T, typename Enable>
void TypeHandler<T, Enable>::from(const T &from_type, Token &token, Serializer &serializer) {
  static const char objectStart[] = "{";
  static const char objectEnd[] = "}";
  token.value_type = Type::ObjectStart;
  token.value = DataRef(objectStart);
  serializer.write(token);
  auto members = Internal::JsonStructBaseDummy<T, T>::js_static_meta_data_info();
  using MembersType = decltype(members);
  Internal::MemberChecker<T, MembersType, 0, MembersType::size - 1>::serializeMembers(from_type, members, token,
                                                                                      serializer, "");
  token.name.size = 0;
  token.name.data = "";
  token.name_type = Type::String;
  token.value_type = Type::ObjectEnd;
  token.value = DataRef(objectEnd);
  serializer.write(token);
}

namespace Internal {
template <typename T, typename F>
struct EnumHandler {
  static inline Error to(T &to_type, ParseContext &context) {
    if (context.token.value_type == Type::String) {
      auto &strings = F::strings();
      for (size_t i = 0; i < strings.size(); i++) {
        const DataRef &ref = strings[i];
        if (ref.size == context.token.value.size) {
          if (memcmp(ref.data, context.token.value.data, ref.size) == 0) {
            to_type = static_cast<T>(i);
            return Error::NoError;
          }
        }
      }
    } else if (context.token.value_type == Type::Number) {
      using enum_int_t = typename std::underlying_type<T>::type;
      enum_int_t tmp;
      auto err = TypeHandler<enum_int_t>::to(tmp, context);
      if (err != Error::NoError)
        return err;
      to_type = static_cast<T>(tmp);
      return Error::NoError;
    }

    return Error::IllegalDataValue;
  }

  static inline void from(const T &from_type, Token &token, Serializer &serializer) {
    size_t i = static_cast<size_t>(from_type);
    token.value = F::strings()[i];
    token.value_type = Type::String;
    serializer.write(token);
  }
};
}  // namespace Internal

namespace Internal {
static void push_back_escape(char current_char, std::string &to_type) {
  static const char escaped_table[] = {'b', 'f', 'n', 'r', 't', '\"', '\\', '/'};
  static const char replace_table[] = {'\b', '\f', '\n', '\r', '\t', '\"', '\\', '/'};
  static_assert(sizeof(escaped_table) == sizeof(replace_table), "Static tables have to be the same.");
  const char *it = static_cast<const char *>(memchr(escaped_table, current_char, sizeof(escaped_table)));
  if (it) {
    to_type.push_back(replace_table[(it - escaped_table)]);
  } else {
    to_type.push_back('\\');
    to_type.push_back(current_char);
  }
}

static void handle_json_escapes_in(const DataRef &ref, std::string &to_type) {
  to_type.reserve(ref.size);
  const char *it = ref.data;
  size_t size = ref.size;
  while (size) {
    const char *next_it = static_cast<const char *>(memchr(it, '\\', size));
    if (!next_it) {
      to_type.insert(to_type.end(), it, it + size);
      break;
    }
    to_type.insert(to_type.end(), it, next_it);
    size -= next_it - it;
    if (!size) {
      break;
    }
    size -= 2;
    const char current_char = *(next_it + 1);
    // we assume utf-8 encoding when this notation is used and parsing into std::string
    if (current_char == 'u')  // hexadecimal escaped unicode character
    {
      // first convert hex ascii digits to values between 0 and 15, then create
      // UTF-8 bit patterns according to https://en.wikipedia.org/wiki/UTF-8
      bool ok = (size >= 4);
      unsigned char hex[4];
      for (int k = 0; ok && k < 4; k++) {
        const char d = *(next_it + k + 2);
        if (d >= '0' && d <= '9')
          hex[k] = (d - '0');
        else if (d >= 'A' && d <= 'F')
          hex[k] = (d - 'A') + 10;
        else if (d >= 'a' && d <= 'f')
          hex[k] = (d - 'a') + 10;
        else
          ok = false;  // stop parsing and revert to fallback
      }
      if (ok) {
        if (hex[0] || hex[1] & 0x08) {
          // code points: 0x0800 .. 0xffff
          to_type.push_back(0xd0 | hex[0]);
          to_type.push_back(0x80 | (hex[1] << 2) | ((hex[2] & 0x0c) >> 2));
          to_type.push_back(0x80 | ((hex[2] & 0x03) << 4) | hex[3]);
        } else if (hex[1] || hex[2] & 0x08) {
          // code points: 0x0080 .. 0x07ff
          to_type.push_back(0xc0 | (hex[1] << 2) | ((hex[2] & 0x0c) >> 2));
          to_type.push_back(0x80 | ((hex[2] & 0x03) << 4) | hex[3]);
        } else {
          // code points: 0x0000 .. 0x007f
          to_type.push_back((hex[2] << 4) | hex[3]);
        }
        it = next_it + 6;  // advance past hex digits
        size -= 4;
      } else {
        // fallback is to simply push characters as is
        to_type.push_back('\\');
        to_type.push_back(current_char);
        it = next_it + 2;
      }
    } else {
      push_back_escape(current_char, to_type);
      it = next_it + 2;
    }
    if (!size)
      break;
  }
}

static DataRef handle_json_escapes_out(const std::string &data, std::string &buffer) {
  int start_index = 0;
  for (size_t i = 0; i < data.size(); i++) {
    const char cur = data[i];
    if (static_cast<uint8_t>(cur) <= uint8_t('\r') || cur == '\"' || cur == '\\') {
      if (buffer.empty()) {
        buffer.reserve(data.size() + 10);
      }
      size_t diff = i - start_index;
      if (diff > 0) {
        buffer.insert(buffer.end(), data.data() + start_index, data.data() + start_index + diff);
      }
      start_index = int(i) + 1;

      switch (cur) {
        case '\b':
          buffer += std::string("\\b");
          break;
        case '\t':
          buffer += std::string("\\t");
          break;
        case '\n':
          buffer += std::string("\\n");
          break;
        case '\f':
          buffer += std::string("\\f");
          break;
        case '\r':
          buffer += std::string("\\r");
          break;
        case '\"':
          buffer += std::string("\\\"");
          break;
        case '\\':
          buffer += std::string("\\\\");
          break;
        default:
          buffer.push_back(cur);
          break;
      }
    }
  }
  if (buffer.size()) {
    size_t diff = data.size() - start_index;
    if (diff > 0) {
      buffer.insert(buffer.end(), data.data() + start_index, data.data() + start_index + diff);
    }
    return DataRef(buffer.data(), buffer.size());
  }
  return DataRef(data.data(), data.size());
}
}  // namespace Internal
/// \private
template <>
struct TypeHandler<std::string> {
  static inline Error to(std::string &to_type, ParseContext &context) {
    to_type.clear();
    Internal::handle_json_escapes_in(context.token.value, to_type);
    return Error::NoError;
  }

  static inline void from(const std::string &str, Token &token, Serializer &serializer) {
    std::string buffer;
    DataRef ref = Internal::handle_json_escapes_out(str, buffer);
    token.value_type = Type::String;
    token.value.data = ref.data;
    token.value.size = ref.size;
    serializer.write(token);
  }
};

namespace Internal {
// This code is taken from https://github.com/jorgen/float_tools
namespace ft {
template <typename T>
struct float_base10 {
  uint8_t negative;
  uint8_t inf;
  uint8_t nan;
  uint8_t significand_digit_count;
  int exp;
  T significand;
};

template <typename T>
struct parsed_string : float_base10<T> {
  const char *endptr;
};

enum class parse_string_error {
  ok,
  invalid_format,
  multiple_commas,
  empty_string,
  illegal_exponent_value
};

constexpr static inline uint64_t high(uint64_t x) {
  return x >> 32;
}
constexpr static inline uint64_t low(uint64_t x) {
  return x & ~uint32_t(0);
}

template <int shift = 1>
inline void left_shift(uint64_t (&a)[2]) {
  static_assert(shift < sizeof(*a) * 8,
                "This functions does only support shifting by sizes smaller than sizeof(*a) * 8");
  a[1] = a[1] << shift | (a[0] >> (int(sizeof(uint64_t) * 8) - shift));
  a[0] = a[0] << shift;
}

template <int shift = 1>
inline void left_shift(uint64_t &a) {
  static_assert(shift < sizeof(a) * 8,
                "This functions does only support shifting by sizes smaller than sizeof(*a) * 8");
  a = a << shift;
}

inline void left_shift(uint64_t (&a)[2], int shift) {
  if (shift > int(sizeof(*a)) * 8) {
    auto shift_0 = (int(sizeof(uint64_t) * 8) - shift);
    if (shift_0 > 0)
      a[1] = a[0] >> shift_0;
    else
      a[1] = a[0] << -shift_0;

    a[0] = 0;
  } else {
    a[1] = a[1] << shift | (a[0] >> (int(sizeof(uint64_t) * 8) - shift));
    a[0] = a[0] << shift;
  }
}

inline void left_shift(uint64_t &a, int shift) {
  a = a << shift;
}

inline void right_shift(uint64_t (&a)[2]) {
  a[0] = a[0] >> 1 | (a[1] << ((sizeof(uint64_t) * 8) - 1));
  a[1] = a[1] >> 1;
}

inline void right_shift(uint64_t &a) {
  a = a >> 1;
}

inline uint64_t mask32(uint64_t a) {
  return a & ((uint64_t(1) << 32) - 1);
}

inline void add(const uint64_t (&a)[2], uint64_t (&b)[2]) {
  uint64_t tmplow[2];
  uint64_t tmphigh[2];
  tmplow[0] = low(a[0]) + low(b[0]);
  tmplow[1] = low(a[1]) + low(b[1]);
  tmphigh[0] = high(a[0]) + high(b[0]);
  tmphigh[1] = high(a[1]) + high(b[1]);

  tmphigh[0] += tmplow[0] >> 32;
  tmplow[1] += tmphigh[0] >> 32;
  tmphigh[1] += tmplow[1] >> 32;

  b[0] = mask32(tmplow[0]) | (tmphigh[0] << 32);
  b[1] = mask32(tmplow[1]) | (tmphigh[1] << 32);
}

inline void add(const uint64_t &a, uint64_t &b) {
  b += a;
}

inline void divide_by_10(uint64_t (&a)[2]) {
  uint64_t remainder = a[1] % 10;
  a[1] /= 10;
  uint64_t high_pluss_reminder = high(a[0]) + (remainder << 32);
  uint64_t high_d = high_pluss_reminder / 10;
  uint64_t high_r = high_pluss_reminder % 10;
  uint64_t low_d = (low(a[0]) + (high_r << 32)) / 10;
  a[0] = high_d << 32 | low_d;
}

inline void divide_by_10(uint64_t &a) {
  a /= 10;
}

template <typename T>
struct float_info {
};

static inline int bit_scan_reverse(uint64_t a) {
  assert(a);
#ifdef _MSC_VER
  unsigned long index;
  #ifdef _WIN64
  _BitScanReverse64(&index, a);
  #else
  if (_BitScanReverse(&index, a >> 32))
    index += 32;
  else
    _BitScanReverse(&index, a & (~uint32_t(0)));
  #endif
  return int(index);
#else
  static_assert(sizeof(unsigned long long) == sizeof(uint64_t), "Wrong size for builtin_clzll");
  return 63 - __builtin_clzll(a);
#endif
}

template <>
struct float_info<double> {
  static inline constexpr int mentissa_width() noexcept {
    return 52;
  }
  static inline constexpr int exponent_width() noexcept {
    return 11;
  }
  static inline constexpr int bias() noexcept {
    return (1 << (exponent_width() - 1)) - 1;
  }
  static inline constexpr int max_base10_exponent() noexcept {
    return 308;
  }
  static inline constexpr int min_base10_exponent() noexcept {
    return -324;
  }
  static inline constexpr int max_double_5_pow_q() noexcept {
    return 23;
  }  // floor(log_5(1 << (mentissawidth + 2)))
  static inline constexpr int max_double_2_pow_q() noexcept {
    return 54;
  }  // floor(log_2(1 << (mentissawidth + 2)))

  using str_to_float_conversion_type = uint64_t[2];
  using uint_alias = uint64_t;
  static inline constexpr int str_to_float_binary_exponent_init() noexcept {
    return 64 + 60;
  }
  static inline constexpr uint64_t str_to_float_mask() noexcept {
    return ~((uint64_t(1) << 60) - 1);
  }
  static inline constexpr uint64_t str_to_float_top_bit_in_mask() noexcept {
    return uint64_t(1) << 63;
  }
  static inline constexpr int str_to_float_expanded_length() noexcept {
    return 19;
  }
  static inline constexpr bool conversion_type_has_mask(const str_to_float_conversion_type &a) noexcept {
    return a[1] & str_to_float_mask();
  }
  static inline constexpr bool conversion_type_has_top_bit_in_mask(const str_to_float_conversion_type &a) noexcept {
    return a[1] & str_to_float_top_bit_in_mask();
  }
  static inline constexpr bool conversion_type_is_null(const str_to_float_conversion_type &a) noexcept {
    return !a[0] && !a[1];
  }
  static inline int shift_left_msb_to_index(str_to_float_conversion_type &a, int index) {
    if (a[1]) {
      int msb = bit_scan_reverse(a[1]);
      int shift_count = index - (msb + 64);
      if (shift_count < 0)
        return 0;
      left_shift(a, shift_count);
      return shift_count;
    } else if (a[0]) {
      int msb = bit_scan_reverse(a[0]);
      int shift_count = index - msb;
      if (shift_count < 0)
        return 0;
      left_shift(a, shift_count);
      return shift_count;
    }
    return 0;
  }
  static inline void copy_denormal_to_type(const str_to_float_conversion_type &a, int binary_exponent, bool negative,
                                           double &to_digit) {
    uint64_t q = a[1];
    int expo_shift = -binary_exponent + 9;
    if (expo_shift) {
      q += uint64_t(1) << (expo_shift - 1);
      q >>= expo_shift;
    }
    if (negative)
      q |= uint64_t(1) << 63;
    memcpy(&to_digit, &q, sizeof(q));
  }

  static inline void copy_normal_to_type(const str_to_float_conversion_type &a, int binary_exponent, bool negative,
                                         double &to_digit) {
    uint64_t q = a[1] & ~str_to_float_mask();
    uint64_t to_round_off = (q & ((uint64_t(1) << 8) - 1));
    bool bigger = to_round_off > (uint64_t(1) << (8 - 1)) || (to_round_off == (uint64_t(1) << (8 - 1)) && a[0]);
    bool tie_odd = (!(q & ((uint64_t(1) << 7) - 1))) && (q & (uint64_t(1) << 8)) && !a[0];
    if (bigger || tie_odd) {
      q += uint64_t(1) << (8 - 1);
    }
    q >>= 8;
    q += uint64_t(binary_exponent) << mentissa_width();
    if (negative)
      q |= uint64_t(1) << 63;
    memcpy(&to_digit, &q, sizeof(q));
  }
};

template <typename T>
inline void get_parts(T f, bool &negative, int &exp, uint64_t &mentissa) {
  uint64_t bits = 0;
  static_assert(sizeof(bits) >= sizeof(f), "Incompatible size");
  memcpy(&bits, &f, sizeof(f));
  exp = int((bits >> float_info<T>::mentissa_width()) & (((uint64_t(1) << float_info<T>::exponent_width()) - 1)));
  mentissa = bits & ((uint64_t(1) << float_info<T>::mentissa_width()) - 1);
  negative = bits >> ((sizeof(f) * 8) - 1);
}

template <typename T>
inline void assign_significand_to_float_conversion_type(const float_base10<T> &significand, uint64_t (&a)[2]) {
  a[0] = significand.significand;
  a[1] = 0;
}

inline void copy_conversion_type(const uint64_t (&a)[2], uint64_t (&b)[2]) {
  memcpy(&b, &a, sizeof(b));
}

template <>
struct float_info<float> {
  static inline constexpr int mentissa_width() noexcept {
    return 23;
  }
  static inline constexpr int exponent_width() noexcept {
    return 8;
  }
  static inline constexpr int bias() noexcept {
    return (1 << (exponent_width() - 1)) - 1;
  }
  static inline constexpr int max_base10_exponent() noexcept {
    return 38;
  }
  static inline constexpr int min_base10_exponent() noexcept {
    return -45;
  }
  static inline constexpr int max_double_5_pow_q() noexcept {
    return 10;
  }  // floor(log_5(1 << (mentissawidth + 2)))
  static inline constexpr int max_double_2_pow_q() noexcept {
    return 25;
  }  // floor(log_2(1 << (mentissawidth + 2)))

  using str_to_float_conversion_type = uint64_t;
  using uint_alias = uint32_t;
  static inline constexpr int str_to_float_binary_exponent_init() noexcept {
    return 60;
  }
  static inline constexpr uint64_t str_to_float_mask() noexcept {
    return ~((uint64_t(1) << 60) - 1);
  }
  static inline constexpr uint64_t str_to_float_top_bit_in_mask() noexcept {
    return uint64_t(1) << 63;
  }
  static inline constexpr int str_to_float_expanded_length() noexcept {
    return 10;
  }
  static inline constexpr bool conversion_type_has_mask(const str_to_float_conversion_type &a) noexcept {
    return a & str_to_float_mask();
  }
  static inline constexpr bool conversion_type_has_top_bit_in_mask(const str_to_float_conversion_type &a) noexcept {
    return a & str_to_float_top_bit_in_mask();
  }
  static inline constexpr bool conversion_type_is_null(const str_to_float_conversion_type &a) noexcept {
    return !a;
  }
  static inline int shift_left_msb_to_index(str_to_float_conversion_type &a, int index) {
    if (a) {
      int msb = bit_scan_reverse(a);
      int shift_count = index - msb;
      if (shift_count < 0)
        return 0;
      left_shift(a, shift_count);
      return shift_count;
    }
    return 0;
  }
  static inline void copy_denormal_to_type(const str_to_float_conversion_type &a, int binary_exponent, bool negative,
                                           float &to_digit) {
    uint64_t q = a;
    int expo_shift = -binary_exponent + 38;
    if (expo_shift) {
      q += uint64_t(1) << (expo_shift - 1);
      q >>= expo_shift;
    }
    if (negative)
      q |= uint64_t(1) << 31;
    uint32_t to_copy = uint32_t(q);
    memcpy(&to_digit, &to_copy, sizeof(to_copy));
  }
  static inline void copy_normal_to_type(const str_to_float_conversion_type &a, int binary_exponent, bool negative,
                                         float &to_digit) {
    uint64_t q = a & ~str_to_float_mask();
    bool bigger = (q & ((uint64_t(1) << 37) - 1)) > (uint64_t(1) << (37 - 1));
    bool tie_odd = (!(q & ((uint64_t(1) << 36) - 1))) && (q & (uint64_t(1) << 37));
    if (bigger || tie_odd) {
      q += (uint64_t(1) << (37 - 1));
    }
    q >>= 37;
    q += uint64_t(binary_exponent) << mentissa_width();
    if (negative)
      q |= uint64_t(1) << 31;
    uint32_t to_copy = uint32_t(q);
    memcpy(&to_digit, &to_copy, sizeof(to_digit));
  }
};

template <typename T>
inline T make_zero(bool negative) {
  using uint_ft = typename float_info<T>::uint_alias;
  uint_ft tmp = 0;
  tmp = uint_ft(negative) << ((sizeof(T) * 8) - 1);
  T ret;
  memcpy(&ret, &tmp, sizeof(ret));
  return ret;
}

template <typename T>
inline T make_inf(bool negative) {
  using uint_ft = typename float_info<T>::uint_alias;
  uint_ft tmp = (uint_ft(1) << float_info<T>::exponent_width()) - 1;
  tmp <<= float_info<T>::mentissa_width();
  tmp += uint_ft(negative) << ((sizeof(T) * 8) - 1);
  T ret;
  memcpy(&ret, &tmp, sizeof(ret));
  return ret;
}

template <typename T>
inline T make_nan(bool positive, uint64_t pos = 1) {
  if (pos == 0)
    pos++;
  using uint_ft = typename float_info<T>::uint_alias;
  uint_ft tmp = (uint_ft(1) << float_info<T>::exponent_width()) - 1;
  tmp <<= float_info<T>::mentissa_width();
  tmp |= pos;
  tmp |= uint_ft(!positive) << ((sizeof(T) * 8) - 1);
  T ret;
  memcpy(&ret, &tmp, sizeof(ret));
  return ret;
}

template <typename T>
inline bool is_nan(T t) {
  bool negative;
  int exp;
  uint64_t mentissa;
  get_parts(t, negative, exp, mentissa);
  return exp == ((int(1) << float_info<T>::exponent_width()) - 1) && mentissa > 0;
}

template <typename T>
inline bool is_inf(T t) {
  bool negative;
  int exp;
  uint64_t mentissa;
  get_parts(t, negative, exp, mentissa);
  return exp == ((int(1) << float_info<T>::exponent_width()) - 1) && mentissa == 0;
}

template <typename T>
const T &max(const T &a, const T &b) {
  return (a < b) ? b : a;
}

template <typename T>
const T &min(const T &a, const T &b) {
  return (b < a) ? b : a;
}

template <typename I, typename P>
I find_if(I first, I last, P p) {
  for (; first != last; ++first) {
    if (p(*first)) {
      return first;
    }
  }
  return last;
}

template <typename T>
inline void assign_significand_to_float_conversion_type(const float_base10<T> &significand, uint64_t &a) {
  a = significand.significand;
}

inline void copy_conversion_type(const uint64_t &a, uint64_t &b) {
  b = a;
}

template <typename T, int COUNT, T SUM>
struct Pow10 {
  static inline T get() noexcept {
    return Pow10<T, COUNT - 1, SUM * T(10)>::get();
  }
};
template <typename T, T SUM>
struct Pow10<T, 1, SUM> {
  static inline T get() noexcept {
    return SUM;
  }
};
template <typename T, T SUM>
struct Pow10<T, 0, SUM> {
  static inline T get() noexcept {
    return 1;
  }
};

template <typename T, T VALUE, int SUM, T ABORT_VALUE, bool CONTINUE>
struct StaticLog10 {
  constexpr static int get() noexcept {
    return StaticLog10<T, VALUE / 10, SUM + 1, ABORT_VALUE, VALUE / 10 != ABORT_VALUE>::get();
  }
};

template <typename T, T VALUE, T ABORT_VALUE, int SUM>
struct StaticLog10<T, VALUE, SUM, ABORT_VALUE, false> {
  constexpr static int get() noexcept {
    return SUM;
  }
};

template <typename T, int WIDTH, int CURRENT>
struct CharsInDigit {
  static int lower_bounds(T t) noexcept {
    if (Pow10<T, CURRENT + WIDTH / 2, 1>::get() - 1 < t) {
      return CharsInDigit<T, WIDTH - (WIDTH / 2 + 1), CURRENT + WIDTH / 2 + 1>::lower_bounds(t);
    }
    return CharsInDigit<T, WIDTH / 2, CURRENT>::lower_bounds(t);
  }
};
template <typename T, int CURRENT>
struct CharsInDigit<T, 0, CURRENT> {
  static int lower_bounds(T) noexcept {
    return CURRENT;
  }
};
template <typename T, int CURRENT>
struct CharsInDigit<T, -1, CURRENT> {
  static int lower_bounds(T) noexcept {
    return CURRENT;
  }
};

template <typename T>
T iabs(typename std::enable_if<std::is_unsigned<T>::value, T>::type a) {
  return a;
}

template <typename T>
T iabs(typename std::enable_if<std::is_signed<T>::value, T>::type a) {
  // this
  if (a > 0)
    return a;
  if (a == std::numeric_limits<T>::min())
    a++;
  return -a;
}

template <typename T>
int count_chars(T t) noexcept {
  if (iabs<T>(t) < T(10))
    return 1;
  constexpr int maxChars = StaticLog10<T, std::numeric_limits<T>::max(), 0, 0, true>::get() + 1;
  return CharsInDigit<T, maxChars, 0>::lower_bounds(iabs<T>(t)) - 1;
}

namespace ryu {
template <typename T>
struct cache_values {
};

template <>
struct cache_values<double> {
  constexpr static const int b0 = 124;
  constexpr static const int b1 = 124;

  static const uint64_t *less_than(int index) {
    static const uint64_t data[326][2] = {{/*  0*/ UINT64_C(0), UINT64_C(1152921504606846976)},
                                          {/*  1*/ UINT64_C(0), UINT64_C(720575940379279360)},
                                          {/*  2*/ UINT64_C(0), UINT64_C(900719925474099200)},
                                          {/*  3*/ UINT64_C(0), UINT64_C(1125899906842624000)},
                                          {/*  4*/ UINT64_C(0), UINT64_C(703687441776640000)},
                                          {/*  5*/ UINT64_C(0), UINT64_C(879609302220800000)},
                                          {/*  6*/ UINT64_C(0), UINT64_C(1099511627776000000)},
                                          {/*  7*/ UINT64_C(0), UINT64_C(687194767360000000)},
                                          {/*  8*/ UINT64_C(0), UINT64_C(858993459200000000)},
                                          {/*  9*/ UINT64_C(0), UINT64_C(1073741824000000000)},
                                          {/* 10*/ UINT64_C(0), UINT64_C(671088640000000000)},
                                          {/* 11*/ UINT64_C(0), UINT64_C(838860800000000000)},
                                          {/* 12*/ UINT64_C(0), UINT64_C(1048576000000000000)},
                                          {/* 13*/ UINT64_C(0), UINT64_C(655360000000000000)},
                                          {/* 14*/ UINT64_C(0), UINT64_C(819200000000000000)},
                                          {/* 15*/ UINT64_C(0), UINT64_C(1024000000000000000)},
                                          {/* 16*/ UINT64_C(0), UINT64_C(640000000000000000)},
                                          {/* 17*/ UINT64_C(0), UINT64_C(800000000000000000)},
                                          {/* 18*/ UINT64_C(0), UINT64_C(1000000000000000000)},
                                          {/* 19*/ UINT64_C(0), UINT64_C(625000000000000000)},
                                          {/* 20*/ UINT64_C(0), UINT64_C(781250000000000000)},
                                          {/* 21*/ UINT64_C(0), UINT64_C(976562500000000000)},
                                          {/* 22*/ UINT64_C(0), UINT64_C(610351562500000000)},
                                          {/* 23*/ UINT64_C(0), UINT64_C(762939453125000000)},
                                          {/* 24*/ UINT64_C(0), UINT64_C(953674316406250000)},
                                          {/* 25*/ UINT64_C(0), UINT64_C(596046447753906250)},
                                          {/* 26*/ UINT64_C(9223372036854775808), UINT64_C(745058059692382812)},
                                          {/* 27*/ UINT64_C(11529215046068469760), UINT64_C(931322574615478515)},
                                          {/* 28*/ UINT64_C(4899916394579099648), UINT64_C(582076609134674072)},
                                          {/* 29*/ UINT64_C(6124895493223874560), UINT64_C(727595761418342590)},
                                          {/* 30*/ UINT64_C(16879491403384619008), UINT64_C(909494701772928237)},
                                          {/* 31*/ UINT64_C(7264306198948610048), UINT64_C(1136868377216160297)},
                                          {/* 32*/ UINT64_C(16069406420411351040), UINT64_C(710542735760100185)},
                                          {/* 33*/ UINT64_C(6251699970232025088), UINT64_C(888178419700125232)},
                                          {/* 34*/ UINT64_C(7814624962790031360), UINT64_C(1110223024625156540)},
                                          {/* 35*/ UINT64_C(14107512638598545408), UINT64_C(693889390390722837)},
                                          {/* 36*/ UINT64_C(3799332742966018048), UINT64_C(867361737988403547)},
                                          {/* 37*/ UINT64_C(137479910280134656), UINT64_C(1084202172485504434)},
                                          {/* 38*/ UINT64_C(4697610962352472064), UINT64_C(677626357803440271)},
                                          {/* 39*/ UINT64_C(1260327684513202176), UINT64_C(847032947254300339)},
                                          {/* 40*/ UINT64_C(15410467660923666432), UINT64_C(1058791184067875423)},
                                          {/* 41*/ UINT64_C(16549071315718373376), UINT64_C(661744490042422139)},
                                          {/* 42*/ UINT64_C(16074653126220578816), UINT64_C(827180612553027674)},
                                          {/* 43*/ UINT64_C(10869944370920947712), UINT64_C(1033975765691284593)},
                                          {/* 44*/ UINT64_C(18322930277894062080), UINT64_C(646234853557052870)},
                                          {/* 45*/ UINT64_C(13680290810512801792), UINT64_C(807793566946316088)},
                                          {/* 46*/ UINT64_C(17100363513141002240), UINT64_C(1009741958682895110)},
                                          {/* 47*/ UINT64_C(6076041177285738496), UINT64_C(631088724176809444)},
                                          {/* 48*/ UINT64_C(7595051471607173120), UINT64_C(788860905221011805)},
                                          {/* 49*/ UINT64_C(14105500357936354304), UINT64_C(986076131526264756)},
                                          {/* 50*/ UINT64_C(18039309760564997248), UINT64_C(616297582203915472)},
                                          {/* 51*/ UINT64_C(4102393126996694944), UINT64_C(770371977754894341)},
                                          {/* 52*/ UINT64_C(9739677427173256584), UINT64_C(962964972193617926)},
                                          {/* 53*/ UINT64_C(1475612373555897461), UINT64_C(601853107621011204)},
                                          {/* 54*/ UINT64_C(1844515466944871826), UINT64_C(752316384526264005)},
                                          {/* 55*/ UINT64_C(6917330352108477686), UINT64_C(940395480657830006)},
                                          {/* 56*/ UINT64_C(18158389525349962266), UINT64_C(587747175411143753)},
                                          {/* 57*/ UINT64_C(8862928851405289120), UINT64_C(734683969263929692)},
                                          {/* 58*/ UINT64_C(11078661064256611401), UINT64_C(918354961579912115)},
                                          {/* 59*/ UINT64_C(9236640311893376347), UINT64_C(1147943701974890144)},
                                          {/* 60*/ UINT64_C(5772900194933360217), UINT64_C(717464813734306340)},
                                          {/* 61*/ UINT64_C(7216125243666700271), UINT64_C(896831017167882925)},
                                          {/* 62*/ UINT64_C(13631842573010763243), UINT64_C(1121038771459853656)},
                                          {/* 63*/ UINT64_C(8519901608131727026), UINT64_C(700649232162408535)},
                                          {/* 64*/ UINT64_C(6038190991737270879), UINT64_C(875811540203010669)},
                                          {/* 65*/ UINT64_C(12159424758098976503), UINT64_C(1094764425253763336)},
                                          {/* 66*/ UINT64_C(7599640473811860314), UINT64_C(684227765783602085)},
                                          {/* 67*/ UINT64_C(14111236610692213297), UINT64_C(855284707229502606)},
                                          {/* 68*/ UINT64_C(8415673726510490813), UINT64_C(1069105884036878258)},
                                          {/* 69*/ UINT64_C(9871482097496444662), UINT64_C(668191177523048911)},
                                          {/* 70*/ UINT64_C(7727666603443167924), UINT64_C(835238971903811139)},
                                          {/* 71*/ UINT64_C(5047897235876572001), UINT64_C(1044048714879763924)},
                                          {/* 72*/ UINT64_C(12378307809277633308), UINT64_C(652530446799852452)},
                                          {/* 73*/ UINT64_C(15472884761597041636), UINT64_C(815663058499815565)},
                                          {/* 74*/ UINT64_C(5506047896714138333), UINT64_C(1019578823124769457)},
                                          {/* 75*/ UINT64_C(14970494981514806218), UINT64_C(637236764452980910)},
                                          {/* 76*/ UINT64_C(9489746690038731964), UINT64_C(796545955566226138)},
                                          {/* 77*/ UINT64_C(2638811325693639147), UINT64_C(995682444457782673)},
                                          {/* 78*/ UINT64_C(13178472124626994227), UINT64_C(622301527786114170)},
                                          {/* 79*/ UINT64_C(7249718118928966976), UINT64_C(777876909732642713)},
                                          {/* 80*/ UINT64_C(13673833667088596624), UINT64_C(972346137165803391)},
                                          {/* 81*/ UINT64_C(15463675069571454746), UINT64_C(607716335728627119)},
                                          {/* 82*/ UINT64_C(14717907818536930528), UINT64_C(759645419660783899)},
                                          {/* 83*/ UINT64_C(13785698754743775257), UINT64_C(949556774575979874)},
                                          {/* 84*/ UINT64_C(13227747740142247439), UINT64_C(593472984109987421)},
                                          {/* 85*/ UINT64_C(2699626619895645587), UINT64_C(741841230137484277)},
                                          {/* 86*/ UINT64_C(7986219293296944888), UINT64_C(927301537671855346)},
                                          {/* 87*/ UINT64_C(9603073076737978459), UINT64_C(579563461044909591)},
                                          {/* 88*/ UINT64_C(7392155327495085170), UINT64_C(724454326306136989)},
                                          {/* 89*/ UINT64_C(13851880177796244366), UINT64_C(905567907882671236)},
                                          {/* 90*/ UINT64_C(17314850222245305458), UINT64_C(1131959884853339045)},
                                          {/* 91*/ UINT64_C(13127624398117009863), UINT64_C(707474928033336903)},
                                          {/* 92*/ UINT64_C(11797844479218874425), UINT64_C(884343660041671129)},
                                          {/* 93*/ UINT64_C(912247543741429319), UINT64_C(1105429575052088912)},
                                          {/* 94*/ UINT64_C(570154714838393324), UINT64_C(690893484407555570)},
                                          {/* 95*/ UINT64_C(9936065430402767463), UINT64_C(863616855509444462)},
                                          {/* 96*/ UINT64_C(3196709751148683521), UINT64_C(1079521069386805578)},
                                          {/* 97*/ UINT64_C(6609629612895315105), UINT64_C(674700668366753486)},
                                          {/* 98*/ UINT64_C(17485409052973919689), UINT64_C(843375835458441857)},
                                          {/* 99*/ UINT64_C(8021703260935235899), UINT64_C(1054219794323052322)},
                                          {/*100*/ UINT64_C(9625250556511910341), UINT64_C(658887371451907701)},
                                          {/*101*/ UINT64_C(16643249214067275830), UINT64_C(823609214314884626)},
                                          {/*102*/ UINT64_C(11580689480729318980), UINT64_C(1029511517893605783)},
                                          {/*103*/ UINT64_C(14155459953096906218), UINT64_C(643444698683503614)},
                                          {/*104*/ UINT64_C(8470952904516356965), UINT64_C(804305873354379518)},
                                          {/*105*/ UINT64_C(1365319093790670398), UINT64_C(1005382341692974398)},
                                          {/*106*/ UINT64_C(14688382488901332711), UINT64_C(628363963558108998)},
                                          {/*107*/ UINT64_C(9137106074271890081), UINT64_C(785454954447636248)},
                                          {/*108*/ UINT64_C(11421382592839862601), UINT64_C(981818693059545310)},
                                          {/*109*/ UINT64_C(2526678102097526221), UINT64_C(613636683162215819)},
                                          {/*110*/ UINT64_C(16993405682904071489), UINT64_C(767045853952769773)},
                                          {/*111*/ UINT64_C(7406699048347925649), UINT64_C(958807317440962217)},
                                          {/*112*/ UINT64_C(16158401951285923291), UINT64_C(599254573400601385)},
                                          {/*113*/ UINT64_C(6362944383825240401), UINT64_C(749068216750751732)},
                                          {/*114*/ UINT64_C(7953680479781550502), UINT64_C(936335270938439665)},
                                          {/*115*/ UINT64_C(16500265345931938823), UINT64_C(585209544336524790)},
                                          {/*116*/ UINT64_C(11401959645560147721), UINT64_C(731511930420655988)},
                                          {/*117*/ UINT64_C(14252449556950184652), UINT64_C(914389913025819985)},
                                          {/*118*/ UINT64_C(3980503890905567103), UINT64_C(1142987391282274982)},
                                          {/*119*/ UINT64_C(16322872987098143151), UINT64_C(714367119551421863)},
                                          {/*120*/ UINT64_C(15791905215445291035), UINT64_C(892958899439277329)},
                                          {/*121*/ UINT64_C(5904823464024450082), UINT64_C(1116198624299096662)},
                                          {/*122*/ UINT64_C(17525572720297445013), UINT64_C(697624140186935413)},
                                          {/*123*/ UINT64_C(8071907845089642554), UINT64_C(872030175233669267)},
                                          {/*124*/ UINT64_C(5478198787934665289), UINT64_C(1090037719042086584)},
                                          {/*125*/ UINT64_C(3423874242459165806), UINT64_C(681273574401304115)},
                                          {/*126*/ UINT64_C(18114900858356120969), UINT64_C(851591968001630143)},
                                          {/*127*/ UINT64_C(18031940054517763307), UINT64_C(1064489960002037679)},
                                          {/*128*/ UINT64_C(18187491561714683923), UINT64_C(665306225001273549)},
                                          {/*129*/ UINT64_C(8899306396861191192), UINT64_C(831632781251591937)},
                                          {/*130*/ UINT64_C(15735819014503876894), UINT64_C(1039540976564489921)},
                                          {/*131*/ UINT64_C(2917357856423841202), UINT64_C(649713110352806201)},
                                          {/*132*/ UINT64_C(8258383338957189407), UINT64_C(812141387941007751)},
                                          {/*133*/ UINT64_C(5711293155269098855), UINT64_C(1015176734926259689)},
                                          {/*134*/ UINT64_C(15098773268111656544), UINT64_C(634485459328912305)},
                                          {/*135*/ UINT64_C(5038408529857406968), UINT64_C(793106824161140382)},
                                          {/*136*/ UINT64_C(15521382699176534519), UINT64_C(991383530201425477)},
                                          {/*137*/ UINT64_C(12006707196199028026), UINT64_C(619614706375890923)},
                                          {/*138*/ UINT64_C(10396697976821397129), UINT64_C(774518382969863654)},
                                          {/*139*/ UINT64_C(3772500434171970603), UINT64_C(968147978712329568)},
                                          {/*140*/ UINT64_C(2357812771357481627), UINT64_C(605092486695205980)},
                                          {/*141*/ UINT64_C(2947265964196852033), UINT64_C(756365608369007475)},
                                          {/*142*/ UINT64_C(17519140510528228754), UINT64_C(945457010461259343)},
                                          {/*143*/ UINT64_C(17866991846721224827), UINT64_C(590910631538287089)},
                                          {/*144*/ UINT64_C(8498681753119367322), UINT64_C(738638289422858862)},
                                          {/*145*/ UINT64_C(1399980154544433344), UINT64_C(923297861778573578)},
                                          {/*146*/ UINT64_C(5486673615017658744), UINT64_C(577061163611608486)},
                                          {/*147*/ UINT64_C(16081714055626849238), UINT64_C(721326454514510607)},
                                          {/*148*/ UINT64_C(15490456551106173644), UINT64_C(901658068143138259)},
                                          {/*149*/ UINT64_C(14751384670455329151), UINT64_C(1127072585178922824)},
                                          {/*150*/ UINT64_C(9219615419034580719), UINT64_C(704420365736826765)},
                                          {/*151*/ UINT64_C(16136205292220613803), UINT64_C(880525457171033456)},
                                          {/*152*/ UINT64_C(1723512541566215638), UINT64_C(1100656821463791821)},
                                          {/*153*/ UINT64_C(3383038347692578726), UINT64_C(687910513414869888)},
                                          {/*154*/ UINT64_C(4228797934615723407), UINT64_C(859888141768587360)},
                                          {/*155*/ UINT64_C(5285997418269654259), UINT64_C(1074860177210734200)},
                                          {/*156*/ UINT64_C(3303748386418533912), UINT64_C(671787610756708875)},
                                          {/*157*/ UINT64_C(17964743538305331102), UINT64_C(839734513445886093)},
                                          {/*158*/ UINT64_C(8620871367599500165), UINT64_C(1049668141807357617)},
                                          {/*159*/ UINT64_C(16917259650818157363), UINT64_C(656042588629598510)},
                                          {/*160*/ UINT64_C(11923202526667920896), UINT64_C(820053235786998138)},
                                          {/*161*/ UINT64_C(5680631121480125312), UINT64_C(1025066544733747673)},
                                          {/*162*/ UINT64_C(15079609496993548080), UINT64_C(640666590458592295)},
                                          {/*163*/ UINT64_C(14237825852814547196), UINT64_C(800833238073240369)},
                                          {/*164*/ UINT64_C(3962224260736020283), UINT64_C(1001041547591550462)},
                                          {/*165*/ UINT64_C(16311448218242176389), UINT64_C(625650967244719038)},
                                          {/*166*/ UINT64_C(11165938235947944678), UINT64_C(782063709055898798)},
                                          {/*167*/ UINT64_C(4734050758080155040), UINT64_C(977579636319873498)},
                                          {/*168*/ UINT64_C(7570467742227484804), UINT64_C(610987272699920936)},
                                          {/*169*/ UINT64_C(9463084677784356005), UINT64_C(763734090874901170)},
                                          {/*170*/ UINT64_C(2605483810375669198), UINT64_C(954667613593626463)},
                                          {/*171*/ UINT64_C(8545956409125875105), UINT64_C(596667258496016539)},
                                          {/*172*/ UINT64_C(6070759492979955977), UINT64_C(745834073120020674)},
                                          {/*173*/ UINT64_C(16811821403079720779), UINT64_C(932292591400025842)},
                                          {/*174*/ UINT64_C(15119074395352213391), UINT64_C(582682869625016151)},
                                          {/*175*/ UINT64_C(14287156975762878835), UINT64_C(728353587031270189)},
                                          {/*176*/ UINT64_C(4023888164421434831), UINT64_C(910441983789087737)},
                                          {/*177*/ UINT64_C(9641546223954181443), UINT64_C(1138052479736359671)},
                                          {/*178*/ UINT64_C(12943495417612445258), UINT64_C(711282799835224794)},
                                          {/*179*/ UINT64_C(6955997235160780765), UINT64_C(889103499794030993)},
                                          {/*180*/ UINT64_C(13306682562378363860), UINT64_C(1111379374742538741)},
                                          {/*181*/ UINT64_C(10622519610700171364), UINT64_C(694612109214086713)},
                                          {/*182*/ UINT64_C(17889835531802602109), UINT64_C(868265136517608391)},
                                          {/*183*/ UINT64_C(17750608396325864733), UINT64_C(1085331420647010489)},
                                          {/*184*/ UINT64_C(4176601220062583602), UINT64_C(678332137904381556)},
                                          {/*185*/ UINT64_C(5220751525078229502), UINT64_C(847915172380476945)},
                                          {/*186*/ UINT64_C(11137625424775174782), UINT64_C(1059893965475596181)},
                                          {/*187*/ UINT64_C(9266858899698178191), UINT64_C(662433728422247613)},
                                          {/*188*/ UINT64_C(16195259643050110642), UINT64_C(828042160527809516)},
                                          {/*189*/ UINT64_C(1797330480103086687), UINT64_C(1035052700659761896)},
                                          {/*190*/ UINT64_C(1123331550064429179), UINT64_C(646907937912351185)},
                                          {/*191*/ UINT64_C(6015850456007924378), UINT64_C(808634922390438981)},
                                          {/*192*/ UINT64_C(12131499088437293377), UINT64_C(1010793652988048726)},
                                          {/*193*/ UINT64_C(2970500911845920456), UINT64_C(631746033117530454)},
                                          {/*194*/ UINT64_C(12936498176662176379), UINT64_C(789682541396913067)},
                                          {/*195*/ UINT64_C(11558936702400332569), UINT64_C(987103176746141334)},
                                          {/*196*/ UINT64_C(2612649420572819952), UINT64_C(616939485466338334)},
                                          {/*197*/ UINT64_C(12489183812570800748), UINT64_C(771174356832922917)},
                                          {/*198*/ UINT64_C(1776421710431337223), UINT64_C(963967946041153647)},
                                          {/*199*/ UINT64_C(8027792596660667620), UINT64_C(602479966275721029)},
                                          {/*200*/ UINT64_C(14646426764253222429), UINT64_C(753099957844651286)},
                                          {/*201*/ UINT64_C(9084661418461752229), UINT64_C(941374947305814108)},
                                          {/*202*/ UINT64_C(14901285423393370951), UINT64_C(588359342066133817)},
                                          {/*203*/ UINT64_C(4791548723959549977), UINT64_C(735449177582667272)},
                                          {/*204*/ UINT64_C(5989435904949437471), UINT64_C(919311471978334090)},
                                          {/*205*/ UINT64_C(16710166918041572647), UINT64_C(1149139339972917612)},
                                          {/*206*/ UINT64_C(1220482286921207096), UINT64_C(718212087483073508)},
                                          {/*207*/ UINT64_C(1525602858651508870), UINT64_C(897765109353841885)},
                                          {/*208*/ UINT64_C(6518689591741773992), UINT64_C(1122206386692302356)},
                                          {/*209*/ UINT64_C(13297553031693384553), UINT64_C(701378991682688972)},
                                          {/*210*/ UINT64_C(16621941289616730691), UINT64_C(876723739603361215)},
                                          {/*211*/ UINT64_C(16165740593593525460), UINT64_C(1095904674504201519)},
                                          {/*212*/ UINT64_C(17021116898637035268), UINT64_C(684940421565125949)},
                                          {/*213*/ UINT64_C(7441338068014130373), UINT64_C(856175526956407437)},
                                          {/*214*/ UINT64_C(13913358603445050871), UINT64_C(1070219408695509296)},
                                          {/*215*/ UINT64_C(8695849127153156794), UINT64_C(668887130434693310)},
                                          {/*216*/ UINT64_C(1646439372086670185), UINT64_C(836108913043366638)},
                                          {/*217*/ UINT64_C(11281421251963113539), UINT64_C(1045136141304208297)},
                                          {/*218*/ UINT64_C(133359254835864106), UINT64_C(653210088315130186)},
                                          {/*219*/ UINT64_C(9390071105399605940), UINT64_C(816512610393912732)},
                                          {/*220*/ UINT64_C(11737588881749507425), UINT64_C(1020640762992390915)},
                                          {/*221*/ UINT64_C(5030150041879748189), UINT64_C(637900476870244322)},
                                          {/*222*/ UINT64_C(15511059589204461044), UINT64_C(797375596087805402)},
                                          {/*223*/ UINT64_C(10165452449650800497), UINT64_C(996719495109756753)},
                                          {/*224*/ UINT64_C(17882622827100220070), UINT64_C(622949684443597970)},
                                          {/*225*/ UINT64_C(13129906497020499280), UINT64_C(778687105554497463)},
                                          {/*226*/ UINT64_C(11800697102848236196), UINT64_C(973358881943121829)},
                                          {/*227*/ UINT64_C(9681278698493841575), UINT64_C(608349301214451143)},
                                          {/*228*/ UINT64_C(7489912354689914064), UINT64_C(760436626518063929)},
                                          {/*229*/ UINT64_C(13974076461789780485), UINT64_C(950545783147579911)},
                                          {/*230*/ UINT64_C(15651326816259694659), UINT64_C(594091114467237444)},
                                          {/*231*/ UINT64_C(1117414446615066707), UINT64_C(742613893084046806)},
                                          {/*232*/ UINT64_C(10620140095123609192), UINT64_C(928267366355058507)},
                                          {/*233*/ UINT64_C(4331744550238561793), UINT64_C(580167103971911567)},
                                          {/*234*/ UINT64_C(802994669370814337), UINT64_C(725208879964889459)},
                                          {/*235*/ UINT64_C(14838801391995681634), UINT64_C(906511099956111823)},
                                          {/*236*/ UINT64_C(13936815721567214139), UINT64_C(1133138874945139779)},
                                          {/*237*/ UINT64_C(6404666816765814884), UINT64_C(708211796840712362)},
                                          {/*238*/ UINT64_C(17229205557812044414), UINT64_C(885264746050890452)},
                                          {/*239*/ UINT64_C(3089762873555503901), UINT64_C(1106580932563613066)},
                                          {/*240*/ UINT64_C(6542787814399577842), UINT64_C(691613082852258166)},
                                          {/*241*/ UINT64_C(17401856804854248111), UINT64_C(864516353565322707)},
                                          {/*242*/ UINT64_C(17140634987640422235), UINT64_C(1080645441956653384)},
                                          {/*243*/ UINT64_C(10712896867275263896), UINT64_C(675403401222908365)},
                                          {/*244*/ UINT64_C(18002807102521467775), UINT64_C(844254251528635456)},
                                          {/*245*/ UINT64_C(4056764804442283102), UINT64_C(1055317814410794321)},
                                          {/*246*/ UINT64_C(14064693048844896699), UINT64_C(659573634006746450)},
                                          {/*247*/ UINT64_C(8357494274201345066), UINT64_C(824467042508433063)},
                                          {/*248*/ UINT64_C(5835181824324293428), UINT64_C(1030583803135541329)},
                                          {/*249*/ UINT64_C(15176203686271153152), UINT64_C(644114876959713330)},
                                          {/*250*/ UINT64_C(9746882570984165633), UINT64_C(805143596199641663)},
                                          {/*251*/ UINT64_C(7571917195302819137), UINT64_C(1006429495249552079)},
                                          {/*252*/ UINT64_C(11649977274705343816), UINT64_C(629018434530970049)},
                                          {/*253*/ UINT64_C(727413538099516059), UINT64_C(786273043163712562)},
                                          {/*254*/ UINT64_C(10132638959479170881), UINT64_C(982841303954640702)},
                                          {/*255*/ UINT64_C(1721213331247093897), UINT64_C(614275814971650439)},
                                          {/*256*/ UINT64_C(15986574719341031083), UINT64_C(767844768714563048)},
                                          {/*257*/ UINT64_C(1536474325466737238), UINT64_C(959805960893203811)},
                                          {/*258*/ UINT64_C(17101197517912568437), UINT64_C(599878725558252381)},
                                          {/*259*/ UINT64_C(7541438842108546835), UINT64_C(749848406947815477)},
                                          {/*260*/ UINT64_C(14038484571063071448), UINT64_C(937310508684769346)},
                                          {/*261*/ UINT64_C(13385738875341807559), UINT64_C(585819067927980841)},
                                          {/*262*/ UINT64_C(2897115538895095736), UINT64_C(732273834909976052)},
                                          {/*263*/ UINT64_C(3621394423618869671), UINT64_C(915342293637470065)},
                                          {/*264*/ UINT64_C(9138429047950974993), UINT64_C(1144177867046837581)},
                                          {/*265*/ UINT64_C(8017361164183053322), UINT64_C(715111166904273488)},
                                          {/*266*/ UINT64_C(10021701455228816653), UINT64_C(893888958630341860)},
                                          {/*267*/ UINT64_C(12527126819036020816), UINT64_C(1117361198287927325)},
                                          {/*268*/ UINT64_C(10135297271111206962), UINT64_C(698350748929954578)},
                                          {/*269*/ UINT64_C(3445749552034232895), UINT64_C(872938436162443223)},
                                          {/*270*/ UINT64_C(18142244995324954830), UINT64_C(1091173045203054028)},
                                          {/*271*/ UINT64_C(2115531085223320961), UINT64_C(681983153251908768)},
                                          {/*272*/ UINT64_C(2644413856529151201), UINT64_C(852478941564885960)},
                                          {/*273*/ UINT64_C(3305517320661439001), UINT64_C(1065598676956107450)},
                                          {/*274*/ UINT64_C(6677634343840787280), UINT64_C(665999173097567156)},
                                          {/*275*/ UINT64_C(8347042929800984100), UINT64_C(832498966371958945)},
                                          {/*276*/ UINT64_C(15045489680678618029), UINT64_C(1040623707964948681)},
                                          {/*277*/ UINT64_C(2485902022783054412), UINT64_C(650389817478092926)},
                                          {/*278*/ UINT64_C(12330749565333593823), UINT64_C(812987271847616157)},
                                          {/*279*/ UINT64_C(1578378901384828567), UINT64_C(1016234089809520197)},
                                          {/*280*/ UINT64_C(3292329822579211806), UINT64_C(635146306130950123)},
                                          {/*281*/ UINT64_C(17950470333506178470), UINT64_C(793932882663687653)},
                                          {/*282*/ UINT64_C(8603029861600559375), UINT64_C(992416103329609567)},
                                          {/*283*/ UINT64_C(12294422691141431465), UINT64_C(620260064581005979)},
                                          {/*284*/ UINT64_C(10756342345499401428), UINT64_C(775325080726257474)},
                                          {/*285*/ UINT64_C(4222055895019475977), UINT64_C(969156350907821843)},
                                          {/*286*/ UINT64_C(332941925173478533), UINT64_C(605722719317388652)},
                                          {/*287*/ UINT64_C(416177406466848167), UINT64_C(757153399146735815)},
                                          {/*288*/ UINT64_C(14355279813365723921), UINT64_C(946441748933419768)},
                                          {/*289*/ UINT64_C(8972049883353577450), UINT64_C(591526093083387355)},
                                          {/*290*/ UINT64_C(6603376335764583909), UINT64_C(739407616354234194)},
                                          {/*291*/ UINT64_C(17477592456560505694), UINT64_C(924259520442792742)},
                                          {/*292*/ UINT64_C(6311809266922928155), UINT64_C(577662200276745464)},
                                          {/*293*/ UINT64_C(7889761583653660193), UINT64_C(722077750345931830)},
                                          {/*294*/ UINT64_C(638829942712299434), UINT64_C(902597187932414788)},
                                          {/*295*/ UINT64_C(798537428390374293), UINT64_C(1128246484915518485)},
                                          {/*296*/ UINT64_C(2804928901957677885), UINT64_C(705154053072199053)},
                                          {/*297*/ UINT64_C(8117847145874485260), UINT64_C(881442566340248816)},
                                          {/*298*/ UINT64_C(10147308932343106575), UINT64_C(1101803207925311020)},
                                          {/*299*/ UINT64_C(15565440119569217417), UINT64_C(688627004953319387)},
                                          {/*300*/ UINT64_C(14845114131034133868), UINT64_C(860783756191649234)},
                                          {/*301*/ UINT64_C(9333020626937891527), UINT64_C(1075979695239561543)},
                                          {/*302*/ UINT64_C(12750666919477264060), UINT64_C(672487309524725964)},
                                          {/*303*/ UINT64_C(15938333649346580075), UINT64_C(840609136905907455)},
                                          {/*304*/ UINT64_C(15311231043255837190), UINT64_C(1050761421132384319)},
                                          {/*305*/ UINT64_C(16487048429675980100), UINT64_C(656725888207740199)},
                                          {/*306*/ UINT64_C(15997124518667587221), UINT64_C(820907360259675249)},
                                          {/*307*/ UINT64_C(6161347593052320314), UINT64_C(1026134200324594062)},
                                          {/*308*/ UINT64_C(17685900300939863908), UINT64_C(641333875202871288)},
                                          {/*309*/ UINT64_C(3660631302465278269), UINT64_C(801667344003589111)},
                                          {/*310*/ UINT64_C(18410847183363761549), UINT64_C(1002084180004486388)},
                                          {/*311*/ UINT64_C(2283407452747575160), UINT64_C(626302612502803993)},
                                          {/*312*/ UINT64_C(7465945334361856854), UINT64_C(782878265628504991)},
                                          {/*313*/ UINT64_C(4720745649524933163), UINT64_C(978597832035631239)},
                                          {/*314*/ UINT64_C(9867995058594165083), UINT64_C(611623645022269524)},
                                          {/*315*/ UINT64_C(12334993823242706354), UINT64_C(764529556277836905)},
                                          {/*316*/ UINT64_C(1583684223771219230), UINT64_C(955661945347296132)},
                                          {/*317*/ UINT64_C(10213174676711787827), UINT64_C(597288715842060082)},
                                          {/*318*/ UINT64_C(3543096309034958976), UINT64_C(746610894802575103)},
                                          {/*319*/ UINT64_C(18263928441575862432), UINT64_C(933263618503218878)},
                                          {/*320*/ UINT64_C(6803269257557526116), UINT64_C(583289761564511799)},
                                          {/*321*/ UINT64_C(3892400553519519741), UINT64_C(729112201955639749)},
                                          {/*322*/ UINT64_C(9477186710326787580), UINT64_C(911390252444549686)},
                                          {/*323*/ UINT64_C(2623111351053708667), UINT64_C(1139237815555687108)},
                                          {/*324*/ UINT64_C(10862816631263343725), UINT64_C(712023634722304442)},
                                          {/*325*/ UINT64_C(4355148752224403848), UINT64_C(890029543402880553)}};
    return &data[index][0];
  }

  static const uint64_t *greater_than_equals(int index) {
    static const uint64_t data[291][2] = {{/*  0*/ UINT64_C(0), UINT64_C(1152921504606846976)},
                                          {/*  1*/ UINT64_C(14757395258967641292), UINT64_C(922337203685477580)},
                                          {/*  2*/ UINT64_C(11805916207174113034), UINT64_C(737869762948382064)},
                                          {/*  3*/ UINT64_C(13134081780481200750), UINT64_C(590295810358705651)},
                                          {/*  4*/ UINT64_C(13635833219286100554), UINT64_C(944473296573929042)},
                                          {/*  5*/ UINT64_C(3529968945945059797), UINT64_C(755578637259143234)},
                                          {/*  6*/ UINT64_C(6513323971497958160), UINT64_C(604462909807314587)},
                                          {/*  7*/ UINT64_C(14110667169138643380), UINT64_C(967140655691703339)},
                                          {/*  8*/ UINT64_C(14977882550052825027), UINT64_C(773712524553362671)},
                                          {/*  9*/ UINT64_C(8292957225300349699), UINT64_C(618970019642690137)},
                                          {/* 10*/ UINT64_C(16958080375222469841), UINT64_C(990352031428304219)},
                                          {/* 11*/ UINT64_C(17255813114919886196), UINT64_C(792281625142643375)},
                                          {/* 12*/ UINT64_C(13804650491935908957), UINT64_C(633825300114114700)},
                                          {/* 13*/ UINT64_C(3640696713387902715), UINT64_C(1014120480182583521)},
                                          {/* 14*/ UINT64_C(17669952629677963465), UINT64_C(811296384146066816)},
                                          {/* 15*/ UINT64_C(10446613289000460449), UINT64_C(649037107316853453)},
                                          {/* 16*/ UINT64_C(13025232447658826395), UINT64_C(1038459371706965525)},
                                          {/* 17*/ UINT64_C(10420185958127061116), UINT64_C(830767497365572420)},
                                          {/* 18*/ UINT64_C(8336148766501648892), UINT64_C(664613997892457936)},
                                          {/* 19*/ UINT64_C(5959140396918817582), UINT64_C(1063382396627932698)},
                                          {/* 20*/ UINT64_C(12146009947018874712), UINT64_C(850705917302346158)},
                                          {/* 21*/ UINT64_C(17095505587098920416), UINT64_C(680564733841876926)},
                                          {/* 22*/ UINT64_C(1527367236164900403), UINT64_C(1088903574147003083)},
                                          {/* 23*/ UINT64_C(8600591418415740969), UINT64_C(871122859317602466)},
                                          {/* 24*/ UINT64_C(3191124319990682452), UINT64_C(696898287454081973)},
                                          {/* 25*/ UINT64_C(1416450097243181600), UINT64_C(1115037259926531157)},
                                          {/* 26*/ UINT64_C(12201206522020276249), UINT64_C(892029807941224925)},
                                          {/* 27*/ UINT64_C(9760965217616220999), UINT64_C(713623846352979940)},
                                          {/* 28*/ UINT64_C(15617544348185953599), UINT64_C(1141798154164767904)},
                                          {/* 29*/ UINT64_C(16183384293290673203), UINT64_C(913438523331814323)},
                                          {/* 30*/ UINT64_C(1878660990406807592), UINT64_C(730750818665451459)},
                                          {/* 31*/ UINT64_C(5192277607067356397), UINT64_C(584600654932361167)},
                                          {/* 32*/ UINT64_C(11996992986049680559), UINT64_C(935361047891777867)},
                                          {/* 33*/ UINT64_C(2218896759355923800), UINT64_C(748288838313422294)},
                                          {/* 34*/ UINT64_C(5464466222226649363), UINT64_C(598631070650737835)},
                                          {/* 35*/ UINT64_C(8743145955562638982), UINT64_C(957809713041180536)},
                                          {/* 36*/ UINT64_C(3305167949708200862), UINT64_C(766247770432944429)},
                                          {/* 37*/ UINT64_C(6333483174508471013), UINT64_C(612998216346355543)},
                                          {/* 38*/ UINT64_C(6444224264471643298), UINT64_C(980797146154168869)},
                                          {/* 39*/ UINT64_C(8844728226319224961), UINT64_C(784637716923335095)},
                                          {/* 40*/ UINT64_C(7075782581055379969), UINT64_C(627710173538668076)},
                                          {/* 41*/ UINT64_C(3942554500204787304), UINT64_C(1004336277661868922)},
                                          {/* 42*/ UINT64_C(14222090044389560813), UINT64_C(803469022129495137)},
                                          {/* 43*/ UINT64_C(3998974406027828004), UINT64_C(642775217703596110)},
                                          {/* 44*/ UINT64_C(6398359049644524806), UINT64_C(1028440348325753776)},
                                          {/* 45*/ UINT64_C(1429338424973709522), UINT64_C(822752278660603021)},
                                          {/* 46*/ UINT64_C(15900865998946608910), UINT64_C(658201822928482416)},
                                          {/* 47*/ UINT64_C(18062687968830753610), UINT64_C(1053122916685571866)},
                                          {/* 48*/ UINT64_C(10760801560322692565), UINT64_C(842498333348457493)},
                                          {/* 49*/ UINT64_C(15987338877741974698), UINT64_C(673998666678765994)},
                                          {/* 50*/ UINT64_C(14511695760161428548), UINT64_C(1078397866686025591)},
                                          {/* 51*/ UINT64_C(7920007793387232515), UINT64_C(862718293348820473)},
                                          {/* 52*/ UINT64_C(13714703864193606658), UINT64_C(690174634679056378)},
                                          {/* 53*/ UINT64_C(18254177367967860330), UINT64_C(1104279415486490205)},
                                          {/* 54*/ UINT64_C(14603341894374288264), UINT64_C(883423532389192164)},
                                          {/* 55*/ UINT64_C(15372022330241340934), UINT64_C(706738825911353731)},
                                          {/* 56*/ UINT64_C(17216538098902324849), UINT64_C(1130782121458165970)},
                                          {/* 57*/ UINT64_C(13773230479121859879), UINT64_C(904625697166532776)},
                                          {/* 58*/ UINT64_C(7329235568555577580), UINT64_C(723700557733226221)},
                                          {/* 59*/ UINT64_C(2174039640102551741), UINT64_C(578960446186580977)},
                                          {/* 60*/ UINT64_C(7167812238905993108), UINT64_C(926336713898529563)},
                                          {/* 61*/ UINT64_C(13112947420608615133), UINT64_C(741069371118823650)},
                                          {/* 62*/ UINT64_C(10490357936486892106), UINT64_C(592855496895058920)},
                                          {/* 63*/ UINT64_C(16784572698379027370), UINT64_C(948568795032094272)},
                                          {/* 64*/ UINT64_C(6048960529219401250), UINT64_C(758855036025675418)},
                                          {/* 65*/ UINT64_C(12217866052859341646), UINT64_C(607084028820540334)},
                                          {/* 66*/ UINT64_C(8480539240349215664), UINT64_C(971334446112864535)},
                                          {/* 67*/ UINT64_C(6784431392279372531), UINT64_C(777067556890291628)},
                                          {/* 68*/ UINT64_C(12806242743307318671), UINT64_C(621654045512233302)},
                                          {/* 69*/ UINT64_C(5732593130324068582), UINT64_C(994646472819573284)},
                                          {/* 70*/ UINT64_C(8275423319001165189), UINT64_C(795717178255658627)},
                                          {/* 71*/ UINT64_C(17688385099426663120), UINT64_C(636573742604526901)},
                                          {/* 72*/ UINT64_C(2475974455889288731), UINT64_C(1018517988167243043)},
                                          {/* 73*/ UINT64_C(9359477194195251631), UINT64_C(814814390533794434)},
                                          {/* 74*/ UINT64_C(11176930570098111628), UINT64_C(651851512427035547)},
                                          {/* 75*/ UINT64_C(3125693653189337312), UINT64_C(1042962419883256876)},
                                          {/* 76*/ UINT64_C(17257950181519111142), UINT64_C(834369935906605500)},
                                          {/* 77*/ UINT64_C(13806360145215288914), UINT64_C(667495948725284400)},
                                          {/* 78*/ UINT64_C(3643432158634910646), UINT64_C(1067993517960455041)},
                                          {/* 79*/ UINT64_C(17672140985875569810), UINT64_C(854394814368364032)},
                                          {/* 80*/ UINT64_C(6759015159216635201), UINT64_C(683515851494691226)},
                                          {/* 81*/ UINT64_C(3435726625262795676), UINT64_C(1093625362391505962)},
                                          {/* 82*/ UINT64_C(13816627744435967510), UINT64_C(874900289913204769)},
                                          {/* 83*/ UINT64_C(14742651010290684331), UINT64_C(699920231930563815)},
                                          {/* 84*/ UINT64_C(5141497542755543314), UINT64_C(1119872371088902105)},
                                          {/* 85*/ UINT64_C(4113198034204434651), UINT64_C(895897896871121684)},
                                          {/* 86*/ UINT64_C(6979907242105458044), UINT64_C(716718317496897347)},
                                          {/* 87*/ UINT64_C(14857200402110643194), UINT64_C(1146749307995035755)},
                                          {/* 88*/ UINT64_C(11885760321688514555), UINT64_C(917399446396028604)},
                                          {/* 89*/ UINT64_C(13197957072092721967), UINT64_C(733919557116822883)},
                                          {/* 90*/ UINT64_C(17937063287157998220), UINT64_C(587135645693458306)},
                                          {/* 91*/ UINT64_C(2873859556259424890), UINT64_C(939417033109533291)},
                                          {/* 92*/ UINT64_C(17056482903975181205), UINT64_C(751533626487626632)},
                                          {/* 93*/ UINT64_C(6266488693696324317), UINT64_C(601226901190101306)},
                                          {/* 94*/ UINT64_C(2647684280430298261), UINT64_C(961963041904162090)},
                                          {/* 95*/ UINT64_C(2118147424344238609), UINT64_C(769570433523329672)},
                                          {/* 96*/ UINT64_C(12762564383701121857), UINT64_C(615656346818663737)},
                                          {/* 97*/ UINT64_C(5662707754954153678), UINT64_C(985050154909861980)},
                                          {/* 98*/ UINT64_C(4530166203963322943), UINT64_C(788040123927889584)},
                                          {/* 99*/ UINT64_C(7313481777912568677), UINT64_C(630432099142311667)},
                                          {/*100*/ UINT64_C(15390919659402020207), UINT64_C(1008691358627698667)},
                                          {/*101*/ UINT64_C(4934038098037795519), UINT64_C(806953086902158934)},
                                          {/*102*/ UINT64_C(7636579293172146738), UINT64_C(645562469521727147)},
                                          {/*103*/ UINT64_C(15907875683817345105), UINT64_C(1032899951234763435)},
                                          {/*104*/ UINT64_C(12726300547053876084), UINT64_C(826319960987810748)},
                                          {/*105*/ UINT64_C(17559738067126921513), UINT64_C(661055968790248598)},
                                          {/*106*/ UINT64_C(5959488018951612482), UINT64_C(1057689550064397758)},
                                          {/*107*/ UINT64_C(12146288044645110632), UINT64_C(846151640051518206)},
                                          {/*108*/ UINT64_C(6027681620974178182), UINT64_C(676921312041214565)},
                                          {/*109*/ UINT64_C(9644290593558685092), UINT64_C(1083074099265943304)},
                                          {/*110*/ UINT64_C(11404781289588858397), UINT64_C(866459279412754643)},
                                          {/*111*/ UINT64_C(16502522661154907364), UINT64_C(693167423530203714)},
                                          {/*112*/ UINT64_C(15335989813622120813), UINT64_C(1109067877648325943)},
                                          {/*113*/ UINT64_C(1200745406671965681), UINT64_C(887254302118660755)},
                                          {/*114*/ UINT64_C(960596325337572544), UINT64_C(709803441694928604)},
                                          {/*115*/ UINT64_C(8915651750023936718), UINT64_C(1135685506711885766)},
                                          {/*116*/ UINT64_C(3443172585277239051), UINT64_C(908548405369508613)},
                                          {/*117*/ UINT64_C(10133235697705611887), UINT64_C(726838724295606890)},
                                          {/*118*/ UINT64_C(8106588558164489509), UINT64_C(581470979436485512)},
                                          {/*119*/ UINT64_C(16659890507805093539), UINT64_C(930353567098376819)},
                                          {/*120*/ UINT64_C(17017261220985985154), UINT64_C(744282853678701455)},
                                          {/*121*/ UINT64_C(13613808976788788123), UINT64_C(595426282942961164)},
                                          {/*122*/ UINT64_C(10714047918636330028), UINT64_C(952682052708737863)},
                                          {/*123*/ UINT64_C(15949935964392884668), UINT64_C(762145642166990290)},
                                          {/*124*/ UINT64_C(12759948771514307735), UINT64_C(609716513733592232)},
                                          {/*125*/ UINT64_C(5658522775455251083), UINT64_C(975546421973747572)},
                                          {/*126*/ UINT64_C(15594864664589931836), UINT64_C(780437137578998057)},
                                          {/*127*/ UINT64_C(5097194102188124822), UINT64_C(624349710063198446)},
                                          {/*128*/ UINT64_C(776812934017179069), UINT64_C(998959536101117514)},
                                          {/*129*/ UINT64_C(4310799161955653579), UINT64_C(799167628880894011)},
                                          {/*130*/ UINT64_C(18206034588532164156), UINT64_C(639334103104715208)},
                                          {/*131*/ UINT64_C(6993562453200000710), UINT64_C(1022934564967544334)},
                                          {/*132*/ UINT64_C(9284198777301910891), UINT64_C(818347651974035467)},
                                          {/*133*/ UINT64_C(48661392357708066), UINT64_C(654678121579228374)},
                                          {/*134*/ UINT64_C(7456555857256153553), UINT64_C(1047484994526765398)},
                                          {/*135*/ UINT64_C(13343942315288743489), UINT64_C(837987995621412318)},
                                          {/*136*/ UINT64_C(18053851481714815437), UINT64_C(670390396497129854)},
                                          {/*137*/ UINT64_C(17818115926517973730), UINT64_C(1072624634395407767)},
                                          {/*138*/ UINT64_C(6875795111730558338), UINT64_C(858099707516326214)},
                                          {/*139*/ UINT64_C(9189984904126356993), UINT64_C(686479766013060971)},
                                          {/*140*/ UINT64_C(7325278217118350543), UINT64_C(1098367625620897554)},
                                          {/*141*/ UINT64_C(9549571388436590758), UINT64_C(878694100496718043)},
                                          {/*142*/ UINT64_C(15018354740233093252), UINT64_C(702955280397374434)},
                                          {/*143*/ UINT64_C(12961321140147218235), UINT64_C(1124728448635799095)},
                                          {/*144*/ UINT64_C(10369056912117774588), UINT64_C(899782758908639276)},
                                          {/*145*/ UINT64_C(4605896714952309347), UINT64_C(719826207126911421)},
                                          {/*146*/ UINT64_C(18437481188149425925), UINT64_C(1151721931403058273)},
                                          {/*147*/ UINT64_C(3681938506293809770), UINT64_C(921377545122446619)},
                                          {/*148*/ UINT64_C(6634899619776958139), UINT64_C(737102036097957295)},
                                          {/*149*/ UINT64_C(5307919695821566511), UINT64_C(589681628878365836)},
                                          {/*150*/ UINT64_C(1113973883830685772), UINT64_C(943490606205385338)},
                                          {/*151*/ UINT64_C(8269876736548369264), UINT64_C(754792484964308270)},
                                          {/*152*/ UINT64_C(6615901389238695411), UINT64_C(603833987971446616)},
                                          {/*153*/ UINT64_C(3206744593298092011), UINT64_C(966134380754314586)},
                                          {/*154*/ UINT64_C(17322790933606114902), UINT64_C(772907504603451668)},
                                          {/*155*/ UINT64_C(2790186302659160952), UINT64_C(618326003682761335)},
                                          {/*156*/ UINT64_C(4464298084254657523), UINT64_C(989321605892418136)},
                                          {/*157*/ UINT64_C(18328833726371367311), UINT64_C(791457284713934508)},
                                          {/*158*/ UINT64_C(3595020536871362879), UINT64_C(633165827771147607)},
                                          {/*159*/ UINT64_C(9441381673736090930), UINT64_C(1013065324433836171)},
                                          {/*160*/ UINT64_C(3863756524246962421), UINT64_C(810452259547068937)},
                                          {/*161*/ UINT64_C(14159051663623300906), UINT64_C(648361807637655149)},
                                          {/*162*/ UINT64_C(11586436217571550481), UINT64_C(1037378892220248239)},
                                          {/*163*/ UINT64_C(12958497788799150708), UINT64_C(829903113776198591)},
                                          {/*164*/ UINT64_C(6677449416297410243), UINT64_C(663922491020958873)},
                                          {/*165*/ UINT64_C(6994570251333946066), UINT64_C(1062275985633534197)},
                                          {/*166*/ UINT64_C(16663702645292887822), UINT64_C(849820788506827357)},
                                          {/*167*/ UINT64_C(5952264486750489611), UINT64_C(679856630805461886)},
                                          {/*168*/ UINT64_C(2144925549316962732), UINT64_C(1087770609288739018)},
                                          {/*169*/ UINT64_C(9094638068937390832), UINT64_C(870216487430991214)},
                                          {/*170*/ UINT64_C(10965059269891822988), UINT64_C(696173189944792971)},
                                          {/*171*/ UINT64_C(10165397202343096135), UINT64_C(1113877103911668754)},
                                          {/*172*/ UINT64_C(11821666576616387231), UINT64_C(891101683129335003)},
                                          {/*173*/ UINT64_C(16836030890776930431), UINT64_C(712881346503468002)},
                                          {/*174*/ UINT64_C(12180254166275447398), UINT64_C(1140610154405548804)},
                                          {/*175*/ UINT64_C(13433552147762268241), UINT64_C(912488123524439043)},
                                          {/*176*/ UINT64_C(18125539347693635239), UINT64_C(729990498819551234)},
                                          {/*177*/ UINT64_C(18189780292896818515), UINT64_C(583992399055640987)},
                                          {/*178*/ UINT64_C(14346253209667268331), UINT64_C(934387838489025580)},
                                          {/*179*/ UINT64_C(11477002567733814665), UINT64_C(747510270791220464)},
                                          {/*180*/ UINT64_C(12870950868928962055), UINT64_C(598008216632976371)},
                                          {/*181*/ UINT64_C(13214823760802518641), UINT64_C(956813146612762194)},
                                          {/*182*/ UINT64_C(14261207823383925236), UINT64_C(765450517290209755)},
                                          {/*183*/ UINT64_C(11408966258707140189), UINT64_C(612360413832167804)},
                                          {/*184*/ UINT64_C(7186299569705693333), UINT64_C(979776662131468487)},
                                          {/*185*/ UINT64_C(16817086099990285636), UINT64_C(783821329705174789)},
                                          {/*186*/ UINT64_C(17143017694734138832), UINT64_C(627057063764139831)},
                                          {/*187*/ UINT64_C(1603386608381249869), UINT64_C(1003291302022623731)},
                                          {/*188*/ UINT64_C(16040104545672641188), UINT64_C(802633041618098984)},
                                          {/*189*/ UINT64_C(16521432451280023273), UINT64_C(642106433294479187)},
                                          {/*190*/ UINT64_C(11676896663080395945), UINT64_C(1027370293271166700)},
                                          {/*191*/ UINT64_C(9341517330464316756), UINT64_C(821896234616933360)},
                                          {/*192*/ UINT64_C(7473213864371453404), UINT64_C(657516987693546688)},
                                          {/*193*/ UINT64_C(8267793368252415124), UINT64_C(1052027180309674701)},
                                          {/*194*/ UINT64_C(2924885879860021776), UINT64_C(841621744247739761)},
                                          {/*195*/ UINT64_C(17097303962855658714), UINT64_C(673297395398191808)},
                                          {/*196*/ UINT64_C(5219593452117592003), UINT64_C(1077275832637106894)},
                                          {/*197*/ UINT64_C(7865023576435983925), UINT64_C(861820666109685515)},
                                          {/*198*/ UINT64_C(6292018861148787140), UINT64_C(689456532887748412)},
                                          {/*199*/ UINT64_C(13756578992579969748), UINT64_C(1103130452620397459)},
                                          {/*200*/ UINT64_C(14694612008805886121), UINT64_C(882504362096317967)},
                                          {/*201*/ UINT64_C(4376991977560888251), UINT64_C(706003489677054374)},
                                          {/*202*/ UINT64_C(14381884793581241848), UINT64_C(1129605583483286998)},
                                          {/*203*/ UINT64_C(437461390639262508), UINT64_C(903684466786629599)},
                                          {/*204*/ UINT64_C(4039317927253320330), UINT64_C(722947573429303679)},
                                          {/*205*/ UINT64_C(6920803156544566587), UINT64_C(578358058743442943)},
                                          {/*206*/ UINT64_C(7383936235729396216), UINT64_C(925372893989508709)},
                                          {/*207*/ UINT64_C(9596497803325427296), UINT64_C(740298315191606967)},
                                          {/*208*/ UINT64_C(298500613176521190), UINT64_C(592238652153285574)},
                                          {/*209*/ UINT64_C(7856298610566254551), UINT64_C(947581843445256918)},
                                          {/*210*/ UINT64_C(13663736517936824287), UINT64_C(758065474756205534)},
                                          {/*211*/ UINT64_C(14620338029091369753), UINT64_C(606452379804964427)},
                                          {/*212*/ UINT64_C(8635145587578550312), UINT64_C(970323807687943084)},
                                          {/*213*/ UINT64_C(10597465284804750573), UINT64_C(776259046150354467)},
                                          {/*214*/ UINT64_C(1099274598359979812), UINT64_C(621007236920283574)},
                                          {/*215*/ UINT64_C(9137536986859788346), UINT64_C(993611579072453718)},
                                          {/*216*/ UINT64_C(14688727218971651323), UINT64_C(794889263257962974)},
                                          {/*217*/ UINT64_C(15440330589919231381), UINT64_C(635911410606370379)},
                                          {/*218*/ UINT64_C(13636482499645039241), UINT64_C(1017458256970192607)},
                                          {/*219*/ UINT64_C(3530488370232210746), UINT64_C(813966605576154086)},
                                          {/*220*/ UINT64_C(17581785955153409890), UINT64_C(651173284460923268)},
                                          {/*221*/ UINT64_C(5994764639793993884), UINT64_C(1041877255137477230)},
                                          {/*222*/ UINT64_C(4795811711835195107), UINT64_C(833501804109981784)},
                                          {/*223*/ UINT64_C(7525998184210066409), UINT64_C(666801443287985427)},
                                          {/*224*/ UINT64_C(15730945909478016578), UINT64_C(1066882309260776683)},
                                          {/*225*/ UINT64_C(1516710283356682293), UINT64_C(853505847408621347)},
                                          {/*226*/ UINT64_C(12281414670911076804), UINT64_C(682804677926897077)},
                                          {/*227*/ UINT64_C(4892868214490081593), UINT64_C(1092487484683035324)},
                                          {/*228*/ UINT64_C(7603643386333975598), UINT64_C(873989987746428259)},
                                          {/*229*/ UINT64_C(9772263523809090801), UINT64_C(699191990197142607)},
                                          {/*230*/ UINT64_C(878226379126903990), UINT64_C(1118707184315428172)},
                                          {/*231*/ UINT64_C(11770627547527254161), UINT64_C(894965747452342537)},
                                          {/*232*/ UINT64_C(2037804408537982682), UINT64_C(715972597961874030)},
                                          {/*233*/ UINT64_C(3260487053660772292), UINT64_C(1145556156738998448)},
                                          {/*234*/ UINT64_C(9987087272412438480), UINT64_C(916444925391198758)},
                                          {/*235*/ UINT64_C(15368367447413771430), UINT64_C(733155940312959006)},
                                          {/*236*/ UINT64_C(8605345143189106821), UINT64_C(586524752250367205)},
                                          {/*237*/ UINT64_C(13768552229102570914), UINT64_C(938439603600587528)},
                                          {/*238*/ UINT64_C(18393539412765877377), UINT64_C(750751682880470022)},
                                          {/*239*/ UINT64_C(7336133900728881255), UINT64_C(600601346304376018)},
                                          {/*240*/ UINT64_C(8048465426424299686), UINT64_C(960962154087001629)},
                                          {/*241*/ UINT64_C(10128121155881350072), UINT64_C(768769723269601303)},
                                          {/*242*/ UINT64_C(15481194554188900704), UINT64_C(615015778615681042)},
                                          {/*243*/ UINT64_C(10012516027734599833), UINT64_C(984025245785089668)},
                                          {/*244*/ UINT64_C(15388710451671500513), UINT64_C(787220196628071734)},
                                          {/*245*/ UINT64_C(16000317176079110734), UINT64_C(629776157302457387)},
                                          {/*246*/ UINT64_C(10843112222758935881), UINT64_C(1007641851683931820)},
                                          {/*247*/ UINT64_C(8674489778207148705), UINT64_C(806113481347145456)},
                                          {/*248*/ UINT64_C(3250243007823808641), UINT64_C(644890785077716365)},
                                          {/*249*/ UINT64_C(5200388812518093825), UINT64_C(1031825256124346184)},
                                          {/*250*/ UINT64_C(7849659864756385383), UINT64_C(825460204899476947)},
                                          {/*251*/ UINT64_C(17347774336030839276), UINT64_C(660368163919581557)},
                                          {/*252*/ UINT64_C(12999043678681701549), UINT64_C(1056589062271330492)},
                                          {/*253*/ UINT64_C(3020537313461540593), UINT64_C(845271249817064394)},
                                          {/*254*/ UINT64_C(6105778665511142797), UINT64_C(676216999853651515)},
                                          {/*255*/ UINT64_C(9769245864817828476), UINT64_C(1081947199765842424)},
                                          {/*256*/ UINT64_C(11504745506596173104), UINT64_C(865557759812673939)},
                                          {/*257*/ UINT64_C(12893145220018848806), UINT64_C(692446207850139151)},
                                          {/*258*/ UINT64_C(13250334722546337444), UINT64_C(1107913932560222642)},
                                          {/*259*/ UINT64_C(3221570148553249309), UINT64_C(886331146048178114)},
                                          {/*260*/ UINT64_C(6266604933584509770), UINT64_C(709064916838542491)},
                                          {/*261*/ UINT64_C(2647870264251394986), UINT64_C(1134503866941667986)},
                                          {/*262*/ UINT64_C(16875691470368757282), UINT64_C(907603093553334388)},
                                          {/*263*/ UINT64_C(2432506732069274856), UINT64_C(726082474842667511)},
                                          {/*264*/ UINT64_C(16703400644623061177), UINT64_C(580865979874134008)},
                                          {/*265*/ UINT64_C(4589348142945435944), UINT64_C(929385567798614414)},
                                          {/*266*/ UINT64_C(7360827329098259079), UINT64_C(743508454238891531)},
                                          {/*267*/ UINT64_C(2199313048536696940), UINT64_C(594806763391113225)},
                                          {/*268*/ UINT64_C(3518900877658715104), UINT64_C(951690821425781160)},
                                          {/*269*/ UINT64_C(2815120702126972083), UINT64_C(761352657140624928)},
                                          {/*270*/ UINT64_C(9630794191185398313), UINT64_C(609082125712499942)},
                                          {/*271*/ UINT64_C(651875446928996008), UINT64_C(974531401139999908)},
                                          {/*272*/ UINT64_C(7900197987027017452), UINT64_C(779625120911999926)},
                                          {/*273*/ UINT64_C(2630809574879703639), UINT64_C(623700096729599941)},
                                          {/*274*/ UINT64_C(15277341764033256792), UINT64_C(997920154767359905)},
                                          {/*275*/ UINT64_C(12221873411226605433), UINT64_C(798336123813887924)},
                                          {/*276*/ UINT64_C(13466847543723194670), UINT64_C(638668899051110339)},
                                          {/*277*/ UINT64_C(10478909625731380502), UINT64_C(1021870238481776543)},
                                          {/*278*/ UINT64_C(15761825330068925048), UINT64_C(817496190785421234)},
                                          {/*279*/ UINT64_C(16298809078797050362), UINT64_C(653996952628336987)},
                                          {/*280*/ UINT64_C(11320699267107639286), UINT64_C(1046395124205339180)},
                                          {/*281*/ UINT64_C(9056559413686111429), UINT64_C(837116099364271344)},
                                          {/*282*/ UINT64_C(10934596345690799466), UINT64_C(669692879491417075)},
                                          {/*283*/ UINT64_C(17495354153105279146), UINT64_C(1071508607186267320)},
                                          {/*284*/ UINT64_C(13996283322484223317), UINT64_C(857206885749013856)},
                                          {/*285*/ UINT64_C(7507677843245468330), UINT64_C(685765508599211085)},
                                          {/*286*/ UINT64_C(12012284549192749328), UINT64_C(1097224813758737736)},
                                          {/*287*/ UINT64_C(5920478824612289139), UINT64_C(877779851006990189)},
                                          {/*288*/ UINT64_C(8425731874431741635), UINT64_C(702223880805592151)},
                                          {/*289*/ UINT64_C(6102473369606965969), UINT64_C(1123558209288947442)},
                                          {/*290*/ UINT64_C(15950025139911303745), UINT64_C(898846567431157953)}};
    return &data[index][0];
  }
};

template <>
struct cache_values<float> {
  constexpr static const int b0 = 59;
  constexpr static const int b1 = 61;

  static const uint64_t *less_than(int index) {
    static const uint64_t data[48] = {
        /*  0*/ UINT64_C(2305843009213693952), /*  1*/ UINT64_C(1441151880758558720),
        /*  2*/ UINT64_C(1801439850948198400), /*  3*/ UINT64_C(2251799813685248000),
        /*  4*/ UINT64_C(1407374883553280000), /*  5*/ UINT64_C(1759218604441600000),
        /*  6*/ UINT64_C(2199023255552000000), /*  7*/ UINT64_C(1374389534720000000),
        /*  8*/ UINT64_C(1717986918400000000), /*  9*/ UINT64_C(2147483648000000000),
        /* 10*/ UINT64_C(1342177280000000000), /* 11*/ UINT64_C(1677721600000000000),
        /* 12*/ UINT64_C(2097152000000000000), /* 13*/ UINT64_C(1310720000000000000),
        /* 14*/ UINT64_C(1638400000000000000), /* 15*/ UINT64_C(2048000000000000000),
        /* 16*/ UINT64_C(1280000000000000000), /* 17*/ UINT64_C(1600000000000000000),
        /* 18*/ UINT64_C(2000000000000000000), /* 19*/ UINT64_C(1250000000000000000),
        /* 20*/ UINT64_C(1562500000000000000), /* 21*/ UINT64_C(1953125000000000000),
        /* 22*/ UINT64_C(1220703125000000000), /* 23*/ UINT64_C(1525878906250000000),
        /* 24*/ UINT64_C(1907348632812500000), /* 25*/ UINT64_C(1192092895507812500),
        /* 26*/ UINT64_C(1490116119384765625), /* 27*/ UINT64_C(1862645149230957031),
        /* 28*/ UINT64_C(1164153218269348144), /* 29*/ UINT64_C(1455191522836685180),
        /* 30*/ UINT64_C(1818989403545856475), /* 31*/ UINT64_C(2273736754432320594),
        /* 32*/ UINT64_C(1421085471520200371), /* 33*/ UINT64_C(1776356839400250464),
        /* 34*/ UINT64_C(2220446049250313080), /* 35*/ UINT64_C(1387778780781445675),
        /* 36*/ UINT64_C(1734723475976807094), /* 37*/ UINT64_C(2168404344971008868),
        /* 38*/ UINT64_C(1355252715606880542), /* 39*/ UINT64_C(1694065894508600678),
        /* 40*/ UINT64_C(2117582368135750847), /* 41*/ UINT64_C(1323488980084844279),
        /* 42*/ UINT64_C(1654361225106055349), /* 43*/ UINT64_C(2067951531382569187),
        /* 44*/ UINT64_C(1292469707114105741), /* 45*/ UINT64_C(1615587133892632177),
        /* 46*/ UINT64_C(2019483917365790221), /* 47*/ UINT64_C(1262177448353618888)};
    return &data[index];
  }

  static const uint64_t *greater_than_equals(int index) {
    static const uint64_t data[30] = {
        /*  0*/ UINT64_C(576460752303423488), /*  1*/ UINT64_C(461168601842738790),
        /*  2*/ UINT64_C(368934881474191032), /*  3*/ UINT64_C(295147905179352825),
        /*  4*/ UINT64_C(472236648286964521), /*  5*/ UINT64_C(377789318629571617),
        /*  6*/ UINT64_C(302231454903657293), /*  7*/ UINT64_C(483570327845851669),
        /*  8*/ UINT64_C(386856262276681335), /*  9*/ UINT64_C(309485009821345068),
        /* 10*/ UINT64_C(495176015714152109), /* 11*/ UINT64_C(396140812571321687),
        /* 12*/ UINT64_C(316912650057057350), /* 13*/ UINT64_C(507060240091291760),
        /* 14*/ UINT64_C(405648192073033408), /* 15*/ UINT64_C(324518553658426726),
        /* 16*/ UINT64_C(519229685853482762), /* 17*/ UINT64_C(415383748682786210),
        /* 18*/ UINT64_C(332306998946228968), /* 19*/ UINT64_C(531691198313966349),
        /* 20*/ UINT64_C(425352958651173079), /* 21*/ UINT64_C(340282366920938463),
        /* 22*/ UINT64_C(544451787073501541), /* 23*/ UINT64_C(435561429658801233),
        /* 24*/ UINT64_C(348449143727040986), /* 25*/ UINT64_C(557518629963265578),
        /* 26*/ UINT64_C(446014903970612462), /* 27*/ UINT64_C(356811923176489970),
        /* 28*/ UINT64_C(570899077082383952), /* 29*/ UINT64_C(456719261665907161)};
    return &data[index];
  }
};

constexpr static const double log_10_2 = 0.30102999566398114;
constexpr static const double log_10_5 = 0.6989700043360189;
constexpr static const double log_2_5 = 2.321928094887362;

template <typename T>
inline void normalize(int &exp, uint64_t &mentissa) {
  if (exp) {
    mentissa += uint64_t(1) << float_info<T>::mentissa_width();
    exp = exp - float_info<T>::bias() - float_info<T>::mentissa_width();
  } else {
    exp = 1 - float_info<T>::bias() - float_info<T>::mentissa_width();
  }
}

inline void compute_shortest(uint64_t a, uint64_t b, uint64_t c, bool accept_smaller, bool accept_larger,
                             bool break_tie_down, int &exponent_adjuster, uint64_t &shortest_base10) {
  int i = 0;
  if (!accept_larger)
    c -= 1;

  bool all_a_zero = true;
  bool all_b_zero = true;
  uint64_t a_next = a / 10;
  uint32_t a_remainder = a % 10;
  uint64_t b_next = b / 10;
  uint32_t b_remainder = b % 10;
  uint64_t c_next = c / 10;
  while (a_next < c_next) {
    a_remainder = a % 10;
    b_remainder = b % 10;

    all_b_zero &= bool(!b_remainder);
    all_a_zero &= bool(!a_remainder);

    a = a_next;
    b = b_next;
    c = c_next;
    a_next = a / 10;
    b_next = b / 10;
    c_next = c / 10;
    i++;
  }
  if (accept_smaller && all_a_zero && a % 10 == 0) {
    while (!(a_next % 10)) {
      b_remainder = b % 10;

      all_b_zero &= bool(!b_remainder);

      a = a_next;
      b = b_next;
      c = c_next;
      a_next = a / 10;
      b_next = b / 10;
      c_next = c / 10;
      i++;
    }
  }
  exponent_adjuster = i;

  bool is_tie = b_remainder == 5 && all_b_zero;
  bool want_to_round_down = b_remainder < 5 || (is_tie && break_tie_down);
  bool round_down = (want_to_round_down && (a != b || all_a_zero)) || (b + 1 > c);
  if (round_down) {
    shortest_base10 = b;
  } else {
    shortest_base10 = b + 1;
  }
}

template <typename T>
inline uint64_t multiply_and_shift(uint64_t a, const uint64_t *b, int shift_right, bool round_up) {
  (void)a;
  (void)b;
  (void)shift_right;
  (void)round_up;
  return 0;
}
template <>
inline uint64_t multiply_and_shift<double>(uint64_t a, const uint64_t *b, int shift_right, bool round_up) {
  uint64_t a0, a1, b0, b1, b2, b3, a0b0, a0b1, a0b2, a0b3, a1b0, a1b1, a1b2, a1b3;
  a0 = low(a);
  a1 = high(a);
  b0 = low(b[0]);
  b1 = high(b[0]);
  b2 = low(b[1]);
  b3 = high(b[1]);

  a0b0 = a0 * b0;
  a0b1 = a0 * b1;
  a0b2 = a0 * b2;
  a0b3 = a0 * b3;
  a1b0 = a1 * b0;
  a1b1 = a1 * b1;
  a1b2 = a1 * b2;
  a1b3 = a1 * b3;

  uint64_t result[6];
  result[0] = low(a0b0);
  result[1] = low(a0b1) + low(a1b0) + high(a0b0);
  result[2] = low(a0b2) + low(a1b1) + high(a0b1) + high(a1b0);
  result[3] = low(a0b3) + low(a1b2) + high(a0b2) + high(a1b1);
  result[4] = a1b3 + high(a0b3) + high(a1b2);

  result[1] += high(result[0]);
  result[2] += high(result[1]);
  result[3] += high(result[2]);
  result[4] += high(result[3]);
  result[5] = high(result[4]);

  uint64_t ret[4];
  ret[0] = low(result[0]) | ((low(result[1]) << 32) + high(result[0]));
  ret[1] = low(result[2]) | (low(result[3]) << 32);
  ret[2] = low(result[4]) | (low(result[5]) << 32);

  int index = shift_right / 64;
  int shift_right_in_index = shift_right - (index * 64);
  if (round_up) {
    if (shift_right_in_index) {
      if (!(ret[index] & (uint64_t(1) << (shift_right_in_index - 1))))
        round_up = false;
    } else {
      if (!(index > 0 && ret[index] & uint64_t(1) << 63))
        round_up = false;
    }
  }
  ret[index] >>= shift_right_in_index;
  ret[index] |= (ret[index + 1] & ((uint64_t(1) << shift_right_in_index) - 1)) << (64 - shift_right_in_index);
  ret[index] += round_up;
  return ret[index];
}

template <>
inline uint64_t multiply_and_shift<float>(uint64_t a, const uint64_t *b, int shift_right, bool round_up) {
  uint64_t a0, a1, b0, b1, a0b0, a0b1, a1b0, a1b1;
  a0 = low(a);
  a1 = high(a);
  b0 = low(*b);
  b1 = high(*b);

  a0b0 = a0 * b0;
  a0b1 = a0 * b1;
  a1b0 = a1 * b0;
  a1b1 = a1 * b1;

  uint64_t result[4] = {};
  result[0] = low(a0b0);
  result[1] = low(a0b1) + low(a1b0) + high(a0b0);
  result[2] = low(a1b1) + high(a0b1) + high(a1b0);
  result[3] = high(a1b1);

  result[1] += high(result[0]);
  result[2] += high(result[1]);
  result[3] += high(result[2]);

  uint64_t ret[4];
  ret[0] = low(result[0]) | ((low(result[1]) << 32) + high(result[0]));
  ret[1] = low(result[2]) | (low(result[3]) << 32);

  int index = shift_right / 64;
  int shift_right_in_index = shift_right - (index * 64);
  if (round_up) {
    if (shift_right_in_index) {
      if (!(ret[index] & (uint64_t(1) << (shift_right_in_index - 1))))
        round_up = false;
    } else {
      if (!(index > 0 && ret[index] & uint64_t(1) << 63))
        round_up = false;
    }
  }
  ret[index] >>= shift_right_in_index;
  ret[index] |= (ret[index + 1] & ((uint64_t(1) << shift_right_in_index) - 1)) << (64 - shift_right_in_index);
  ret[index] += round_up;
  return ret[index];
}

inline uint64_t pow_int(int n, int exp) {
  if (!exp)
    return 1;
  uint64_t ret = uint64_t(n);
  for (int i = 0; i < exp; i++) {
    ret *= ret;
  }
  return ret;
}

template <typename T, typename SignificandType>
static float_base10<SignificandType> decode(T f) {
  bool negative;
  int exp;
  uint64_t mentissa;
  get_parts(f, negative, exp, mentissa);
  bool shift_u_with_one = mentissa == 0 && exp > 1;

  if (is_nan(f)) {
    return {negative, false, true, 0, 0, 0};
  }
  if (is_inf(f)) {
    return {negative, true, false, 0, 0, 0};
  }
  if (!exp && !mentissa) {
    return {negative, false, false, 1, 0, 0};
  }

  bool accept_larger = (mentissa % 2 == 0);
  bool accept_smaller = accept_larger;

  normalize<T>(exp, mentissa);

  exp -= 2;
  mentissa *= 4;

  uint64_t u = mentissa;
  if (shift_u_with_one)
    u -= 1;
  else
    u -= 2;

  uint64_t w = mentissa + 2;

  int e10 = exp < 0 ? exp : 0;

  int q;
  int shift_right;
  bool zero[3] = {};
  if (exp >= 0) {
    q = max(0, int(exp * log_10_2) - 1);
    int k = cache_values<T>::b0 + int(q * log_2_5);
    shift_right = -exp + q + k;
    if (q - 1 <= float_info<T>::max_double_5_pow_q()) {
      uint64_t mod = pow_int(5, q - 1);
      if (mod) {
        zero[1] = (mentissa % mod) == 0;
      }
      if (q <= float_info<T>::max_double_5_pow_q()) {
        mod = pow_int(5, q);
        zero[0] = (u % mod) == 0;
        zero[2] = (w % mod) == 0;
      }
    }
  } else {
    q = max(0, int(-exp * log_10_5) - 1);
    int k = int(std::ceil((double(-exp) - double(q)) * log_2_5)) - cache_values<T>::b1;
    shift_right = q - k;
    if (q && q - 1 <= float_info<T>::max_double_2_pow_q()) {
      uint64_t mod = uint64_t(1) << int(q - 1);
      zero[1] = (mentissa % mod) == 0;

      if (q <= float_info<T>::max_double_2_pow_q()) {
        mod <<= 1;
        if (mod) {
          zero[0] = (u % mod) == 0;
          zero[2] = (w % mod) == 0;
        }
      }
    }
  }
  auto cache_value = exp >= 0 ? cache_values<T>::greater_than_equals(q) : cache_values<T>::less_than(-exp - q);
  uint64_t a = multiply_and_shift<T>(u, cache_value, shift_right, true);
  uint64_t b = multiply_and_shift<T>(mentissa, cache_value, shift_right, false);
  uint64_t c = multiply_and_shift<T>(w, cache_value, shift_right, false);

  int exponent_adjust;
  uint64_t shortest_base10;
  compute_shortest(a, b, c, accept_smaller && zero[0], accept_larger || !zero[2], zero[1], exponent_adjust,
                   shortest_base10);
  int significand_digit_count = count_chars(shortest_base10);
  int e = exponent_adjust + e10 + q;
  return {negative, false, false, uint8_t(significand_digit_count), e, shortest_base10};
}

template <typename T>
inline int convert_parsed_to_buffer(const float_base10<T> &result, char *buffer, int buffer_size,
                                    int max_expanded_length, int *digits_truncated = nullptr) {
  if (buffer_size < 1)
    return 0;
  int offset = 0;
  if (result.nan) {
    if (buffer_size >= 3) {
      buffer[offset++] = 'n';
      buffer[offset++] = 'a';
      buffer[offset++] = 'n';
    }
    return offset;
  }

  if (result.negative) {
    buffer[offset++] = '-';
    buffer_size--;
  }

  if (result.inf) {
    if (buffer_size >= 3) {
      buffer[offset++] = 'i';
      buffer[offset++] = 'n';
      buffer[offset++] = 'f';
    }
    return offset;
  }

  char significan_buffer[17] = {};
  assert(result.significand_digit_count <= uint8_t(17));
  int digits_before_decimals = result.significand_digit_count + result.exp;
  int digits_after_decimals = result.exp < 0 ? -result.exp : 0;
  int complete_digits = max(1, digits_before_decimals) + max(1, digits_after_decimals) + 1;
  if (complete_digits < max_expanded_length) {
    char *target_buffer = buffer + offset;
    uint64_t significand = result.significand;
    bool print_desimal_seperator = true;
    if (buffer_size < complete_digits) {
      int to_remove = complete_digits - buffer_size;
      if (digits_truncated)
        *digits_truncated = to_remove;

      int to_remove_after_decimals = std::min(to_remove, digits_after_decimals);
      for (int i = 0; i < to_remove_after_decimals; i++) {
        complete_digits--;
        digits_after_decimals--;
        significand /= 10;
      }
      to_remove -= to_remove_after_decimals;
      if (to_remove > 0) {
        print_desimal_seperator = false;
        if (!digits_after_decimals) {
          complete_digits--;
          to_remove--;
        }
        complete_digits--;
        to_remove--;
        if (to_remove > 0) {
          int to_remove_before_decimals = std::min(to_remove, digits_before_decimals);
          for (int i = 0; i < to_remove_before_decimals; i++) {
            complete_digits--;
            digits_before_decimals--;
            significand /= 10;
          }
        }
      } else if (to_remove == 0 && digits_after_decimals == 0) {
        print_desimal_seperator = false;
        complete_digits--;
      }
    }
    int index_pos = std::max(complete_digits - 1, 0);
    for (int i = 0; i < digits_after_decimals; i++, index_pos--) {
      char remainder = char(significand % 10);
      significand /= 10;
      target_buffer[index_pos] = '0' + remainder;
    }
    if (print_desimal_seperator) {
      if (digits_after_decimals == 0) {
        target_buffer[index_pos--] = '0';
      }
      target_buffer[index_pos--] = '.';
    }
    int add_zeros_before_decimal = std::max(result.exp, 0);
    for (int i = 0; i < add_zeros_before_decimal; i++, index_pos--) {
      target_buffer[index_pos] = '0';
      digits_before_decimals--;
    }
    for (int i = 0; i < digits_before_decimals; i++, index_pos--) {
      char remainder = char(significand % 10);
      significand /= 10;
      target_buffer[index_pos] = '0' + remainder;
    }
    if (digits_before_decimals <= 0)
      target_buffer[index_pos] = '0';
    return complete_digits + offset;
  } else {
    uint64_t significand = result.significand;
    int exp = result.exp;
    for (int i = 0; i < result.significand_digit_count; i++) {
      significan_buffer[result.significand_digit_count - i - 1] = '0' + significand % 10;
      significand /= 10;
    }

    exp += result.significand_digit_count;
    exp--;
    char exponent_buffer[4] = {};
    int exponent_digit_count = count_chars(exp);
    if (exp < 0) {
      exponent_buffer[0] = '-';
    }
    int abs_exp = std::abs(exp);
    for (int i = 0; i < exponent_digit_count; i++) {
      exponent_buffer[exponent_digit_count + (exp < 0) - i - 1] = '0' + abs_exp % 10;
      abs_exp /= 10;
    }
    exponent_digit_count += exp < 0;

    if (offset < buffer_size)
      buffer[offset++] = significan_buffer[0];
    else
      return offset;

    if (result.significand_digit_count > 1) {
      if (offset < buffer_size)
        buffer[offset++] = '.';
      else
        return offset;
    }
    int to_copy = min(buffer_size - offset, int(result.significand_digit_count) - 1);
    for (int i = 0; i < to_copy; i++) {
      buffer[offset++] = significan_buffer[1 + i];
    }

    if (offset >= buffer_size)
      return offset;

    buffer[offset++] = 'e';

    to_copy = min(buffer_size - offset, exponent_digit_count);
    for (int i = 0; i < to_copy; i++) {
      buffer[offset++] = exponent_buffer[i];
    }
  }

  return offset;
}

}  // namespace ryu

template <typename T>
struct set_end_ptr {
  set_end_ptr(parsed_string<T> &parsedString, const char *&current)
      : parsedString(parsedString), current(current) {
  }
  ~set_end_ptr() {
    parsedString.endptr = current;
  }
  parsed_string<T> &parsedString;
  const char *&current;
};

inline bool is_space(char a) {
  if (a == 0x20 || a == 0x09 || a == 0x0a || a == 0x0b || a == 0x0c || a == 0x0d)
    return true;
  return false;
}

template <typename T, bool NoDigitCount>
inline parse_string_error parseNumber(const char *number, size_t size, parsed_string<T> &parsedString) {
  const char *current;
  set_end_ptr<T> setendptr(parsedString, current);
  int desimal_position = -1;
  bool increase_significand = true;

  parsedString.negative = false;
  parsedString.inf = 0;
  parsedString.nan = 0;
  parsedString.significand_digit_count = 0;
  parsedString.significand = 0;
  parsedString.exp = 0;

  const char *number_end = number + size;
  current = find_if(number, number_end, [](const char a) { return !is_space(a); });
  if (number_end == current) {
    return parse_string_error::empty_string;
  }
  if (*current == '-') {
    parsedString.negative = true;
    current++;
  }
  while (current < number_end) {
    if ((*current < '0' || *current > '9') && *current != '.')
      break;

    if (*current == '.') {
      if (desimal_position >= 0)
        return parse_string_error::multiple_commas;
      desimal_position = parsedString.significand_digit_count;
    } else {
#ifdef _MSC_VER
      bool localDigitCount = NoDigitCount;
      if (localDigitCount || parsedString.significand_digit_count < 19)
#else
      if (NoDigitCount || parsedString.significand_digit_count < 19)
#endif
      {
        parsedString.significand = parsedString.significand * T(10) + T(int(*current) - '0');
        parsedString.significand_digit_count++;
      } else if (increase_significand && parsedString.significand_digit_count < 20) {
        increase_significand = false;
        uint64_t digit = uint64_t(*current) - '0';
        static_assert(NoDigitCount || std::is_same<T, uint64_t>::value,
                      "When NoDigitCount is used the significand type has to be uint64_t");
        auto biggest_multiplier = (std::numeric_limits<uint64_t>::max() - digit) / parsedString.significand;

        if (biggest_multiplier >= 10) {
          parsedString.significand = parsedString.significand * T(10) + T(digit);
          parsedString.significand_digit_count++;
        }
      }
    }
    current++;
  }
  if (*current != 'e' && *current != 'E') {
    if (desimal_position >= 0)
      parsedString.exp = desimal_position - parsedString.significand_digit_count;
    else
      parsedString.exp = 0;
    return parse_string_error::ok;
  }
  current++;
  if (current == number_end) {
    return parse_string_error::illegal_exponent_value;
  }
  bool exponent_nagative = false;
  if (*current == '-') {
    exponent_nagative = true;
    current++;
  } else if (*current == '+') {
    current++;
  }
  if (current == number_end) {
    return parse_string_error::illegal_exponent_value;
  }
  int exponent = 0;
  bool exponent_assigned = false;
  while (current < number_end) {
    if ((*current < '0' || *current > '9'))
      break;
    exponent_assigned = true;
    exponent = exponent * 10 + (*current - '0');
    current++;
  }
  if (!exponent_assigned) {
    return parse_string_error::illegal_exponent_value;
  }

  if (exponent_nagative)
    exponent = -exponent;

  if (desimal_position >= 0)
    parsedString.exp = desimal_position - parsedString.significand_digit_count + exponent;
  else
    parsedString.exp = exponent;
  return parse_string_error::ok;
}

inline uint64_t getPow10(uint32_t pow) {
  static uint64_t data[] = {UINT64_C(1),
                            UINT64_C(10),
                            UINT64_C(100),
                            UINT64_C(1000),
                            UINT64_C(10000),
                            UINT64_C(100000),
                            UINT64_C(1000000),
                            UINT64_C(10000000),
                            UINT64_C(100000000),
                            UINT64_C(1000000000),
                            UINT64_C(10000000000),
                            UINT64_C(100000000000),
                            UINT64_C(1000000000000),
                            UINT64_C(10000000000000),
                            UINT64_C(100000000000000),
                            UINT64_C(1000000000000000),
                            UINT64_C(10000000000000000),
                            UINT64_C(100000000000000000),
                            UINT64_C(1000000000000000000),
                            UINT64_C(10000000000000000000)};
  return data[pow];
}

template <typename T, typename SignificandType>
inline T convertToNumber(const parsed_string<SignificandType> &parsed) {
  int base10exponent = parsed.exp + parsed.significand_digit_count - 1;
  if (base10exponent > float_info<T>::max_base10_exponent()) {
    return make_inf<T>(parsed.negative);
  } else if (base10exponent < float_info<T>::min_base10_exponent()) {
    return make_zero<T>(parsed.negative);
  }
  if (parsed.significand == 0) {
    return make_zero<T>(parsed.negative);
  }

#if 1
  if (parsed.significand < ((uint64_t(1) << 53)) && iabs<int>(parsed.exp) < count_chars((uint64_t(1) << 53))) {
    double ds(double(parsed.significand));
    double de(double(getPow10(iabs<int>(parsed.exp))));
    if (parsed.negative)
      ds = -ds;
    return parsed.exp < 0 ? T(ds / de) : T(ds * de);
  }
#endif

  using uint_conversion_type = typename float_info<T>::str_to_float_conversion_type;
  uint_conversion_type a;
  uint_conversion_type b;
  assign_significand_to_float_conversion_type(parsed, a);
  int desimal_exponent = parsed.exp;
  auto binary_exponent = float_info<T>::str_to_float_binary_exponent_init();
  for (; desimal_exponent > 0; desimal_exponent--) {
    left_shift(a);
    copy_conversion_type(a, b);
    left_shift<2>(b);
    add(b, a);

    while (float_info<T>::conversion_type_has_mask(a)) {
      right_shift(a);
      binary_exponent++;
    }
  }

  for (; desimal_exponent < 0; desimal_exponent++) {
    binary_exponent -= float_info<T>::shift_left_msb_to_index(a, float_info<T>::str_to_float_binary_exponent_init());

    divide_by_10(a);
  }

  binary_exponent -= float_info<T>::shift_left_msb_to_index(a, float_info<T>::str_to_float_binary_exponent_init());

  binary_exponent += float_info<T>::bias();
  T to_digit;
  if (binary_exponent <= 0) {
    float_info<T>::copy_denormal_to_type(a, binary_exponent, parsed.negative, to_digit);
  } else if (binary_exponent < (int(1) << float_info<T>::exponent_width()) - 1) {
    float_info<T>::copy_normal_to_type(a, binary_exponent, parsed.negative, to_digit);
  } else {
    to_digit = make_inf<T>(parsed.negative);
  }
  return to_digit;
}

namespace ryu {
template <typename T>
int to_buffer(T d, char *buffer, int buffer_size, int *digits_truncated = nullptr) {
  auto decoded = decode<T, uint64_t>(d);
  return convert_parsed_to_buffer(decoded, buffer, buffer_size, float_info<T>::str_to_float_expanded_length(),
                                  digits_truncated);
}

template <typename T>
inline std::string to_string(T f) {
  auto decoded = decode<T, uint64_t>(f);
  std::string ret;
  ret.resize(25);
  ret.resize(
      size_t(convert_parsed_to_buffer(decoded, &ret[0], int(ret.size()), float_info<T>::str_to_float_expanded_length())));
  return ret;
}
}  // namespace ryu

namespace integer {
template <typename T>
inline int to_buffer(T integer, char *buffer, int buffer_size, int *digits_truncated = nullptr) {
  static_assert(std::is_integral<T>::value, "Tryint to convert non int to string");
  int chars_to_write = ft::count_chars(integer);
  char *target_buffer = buffer;
  bool negative = false;
  if (std::is_signed<T>::value) {
    if (integer < 0) {
      target_buffer[0] = '-';
      target_buffer++;
      buffer_size--;
      negative = true;
    }
  }
  int to_remove = chars_to_write - buffer_size;
  if (to_remove > 0) {
    for (int i = 0; i < to_remove; i++) {
      integer /= 10;
    }
    if (digits_truncated)
      *digits_truncated = to_remove;
    chars_to_write -= to_remove;
  } else if (digits_truncated)
    *digits_truncated = 0;

  for (int i = 0; i < chars_to_write; i++) {
    int remainder = integer % 10;
    if (std::is_signed<T>::value) {
      if (negative)
        remainder = -remainder;
    }
    integer /= 10;
    target_buffer[chars_to_write - 1 - i] = '0' + char(remainder);
  }

  return chars_to_write + negative;
}

template <typename T, typename SignificandType>
inline typename std::enable_if<std::is_signed<T>::value, T>::type make_integer_return_value(SignificandType significand,
                                                                                            bool negative) {
  return negative ? -T(significand) : T(significand);
}

template <typename T, typename SignificandType>
inline typename std::enable_if<std::is_unsigned<T>::value, T>::type make_integer_return_value(
    SignificandType significand, bool) {
  return T(significand);
}

template <typename T, typename SignificandType>
inline T convert_to_integer(const parsed_string<SignificandType> &parsed) {
  if (parsed.inf)
    return parsed.negative ? std::numeric_limits<T>::min() : std::numeric_limits<T>::max();
  if (parsed.nan)
    return T(0);

  int exp = parsed.exp;
  auto significand = parsed.significand;
  if (exp < 0) {
    int chars_in_sig = count_chars(significand);
    if (-exp >= chars_in_sig)
      return T(0);
    while (exp < 0) {
      significand /= 10;
      exp++;
    }
  } else if (exp > 0) {
    int chars_in_sig = count_chars(significand);
    if (exp > ft::StaticLog10<T, std::numeric_limits<T>::max(), 0, 0, true>::get() - chars_in_sig)
      return parsed.negative ? std::numeric_limits<T>::min() : std::numeric_limits<T>::max();
    while (exp > 0) {
      significand *= 10;
      exp--;
    }
  }
  return make_integer_return_value<T>(significand, bool(parsed.negative));
}

template <typename T>
inline parse_string_error to_integer(const char *str, size_t size, T &target, const char *(&endptr)) {
  using SignificandType = typename std::make_unsigned<T>::type;
  parsed_string<SignificandType> ps;
  auto parseResult = parseNumber<SignificandType, true>(str, size, ps);
  endptr = ps.endptr;
  if (parseResult != parse_string_error::ok) {
    target = 0;
  } else {
    target = convert_to_integer<T>(ps);
  }
  return parseResult;
}

template <typename T>
inline parse_string_error to_integer(const std::string &str, T &target, const char *(&endptr)) {
  return to_integer(str.c_str(), str.size(), target, endptr);
}
}  // namespace integer

template <typename T>
inline parse_string_error to_ieee_t(const char *str, size_t size, T &target, const char *(&endptr)) {
  parsed_string<uint64_t> ps;
  auto parseResult = parseNumber<uint64_t, false>(str, size, ps);
  endptr = ps.endptr;
  if (parseResult != parse_string_error::ok) {
    target = make_nan<T>(true, 1);
  } else {
    target = convertToNumber<T>(ps);
  }
  return parseResult;
}

inline parse_string_error to_float(const char *str, size_t size, float &target, const char *(&endptr)) {
  return to_ieee_t(str, size, target, endptr);
}

inline parse_string_error to_double(const char *str, size_t size, double &target, const char *(&endptr)) {
  return to_ieee_t(str, size, target, endptr);
}

}  // namespace ft
}  // namespace Internal
/// \private
template <>
struct TypeHandler<double> {
  static inline Error to(double &to_type, ParseContext &context) {
    const char *pointer;
    auto result = Internal::ft::to_double(context.token.value.data, context.token.value.size, to_type, pointer);
    if (result != Internal::ft::parse_string_error::ok ||
        context.token.value.data + context.token.value.size != pointer)
      return Error::FailedToParseDouble;
    return Error::NoError;
  }

  static inline void from(const double &d, Token &token, Serializer &serializer) {
    // char buf[1/*'-'*/ + (DBL_MAX_10_EXP+1)/*308+1 digits*/ + 1/*'.'*/ + 6/*Default? precision*/ + 1/*\0*/];
    char buf[32];
    int size;
    size = Internal::ft::ryu::to_buffer(d, buf, sizeof(buf));

    if (size <= 0) {
      return;
    }

    token.value_type = Type::Number;
    token.value.data = buf;
    token.value.size = size_t(size);
    serializer.write(token);
  }
};

/// \private
template <>
struct TypeHandler<float> {
  static inline Error to(float &to_type, ParseContext &context) {
    const char *pointer;
    auto result = Internal::ft::to_float(context.token.value.data, context.token.value.size, to_type, pointer);
    if (result != Internal::ft::parse_string_error::ok ||
        context.token.value.data + context.token.value.size != pointer)
      return Error::FailedToParseFloat;
    return Error::NoError;
  }

  static inline void from(const float &f, Token &token, Serializer &serializer) {
    char buf[16];
    int size;
    size = Internal::ft::ryu::to_buffer(f, buf, sizeof(buf));
    if (size < 0) {
      return;
    }

    token.value_type = Type::Number;
    token.value.data = buf;
    token.value.size = size_t(size);
    serializer.write(token);
  }
};

/// \private
template <typename T>
struct TypeHandlerIntType {
  static inline Error to(T &to_type, ParseContext &context) {
    const char *pointer;
    auto parse_error =
        Internal::ft::integer::to_integer(context.token.value.data, context.token.value.size, to_type, pointer);
    if (parse_error != Internal::ft::parse_string_error::ok || context.token.value.data == pointer)
      return Error::FailedToParseInt;
    return Error::NoError;
  }

  static inline void from(const T &from_type, Token &token, Serializer &serializer) {
    char buf[40];
    int digits_truncated;
    int size = Internal::ft::integer::to_buffer(from_type, buf, sizeof(buf), &digits_truncated);
    if (size <= 0 || digits_truncated) {
      fprintf(stderr, "error serializing int token\n");
      return;
    }

    token.value_type = Type::Number;
    token.value.data = buf;
    token.value.size = size_t(size);
    serializer.write(token);
  }
};

/// \private
template <>
struct TypeHandler<short int> : TypeHandlerIntType<short int> {
};

/// \private
template <>
struct TypeHandler<unsigned short int> : TypeHandlerIntType<unsigned short int> {
};

/// \private
template <>
struct TypeHandler<int> : TypeHandlerIntType<int> {
};

/// \private
template <>
struct TypeHandler<unsigned int> : TypeHandlerIntType<unsigned int> {
};

/// \private
template <>
struct TypeHandler<long int> : TypeHandlerIntType<long int> {
};

/// \private
template <>
struct TypeHandler<unsigned long int> : TypeHandlerIntType<unsigned long int> {
};

/// \private
template <>
struct TypeHandler<long long int> : TypeHandlerIntType<long long int> {
};

/// \private
template <>
struct TypeHandler<unsigned long long int> : TypeHandlerIntType<unsigned long long int> {
};

template <>
struct TypeHandler<uint8_t> : TypeHandlerIntType<uint8_t> {
};

template <>
struct TypeHandler<int8_t> : TypeHandlerIntType<int8_t> {
};

template <>
struct TypeHandler<char> : TypeHandlerIntType<char> {
};

/// \private
template <typename T>
struct TypeHandler<Nullable<T>> {
 public:
  static inline Error to(Nullable<T> &to_type, ParseContext &context) {
    if (context.token.value_type == Type::Null)
      return Error::NoError;
    return TypeHandler<T>::to(to_type.data, context);
  }

  static inline void from(const Nullable<T> &opt, Token &token, Serializer &serializer) {
    TypeHandler<T>::from(opt(), token, serializer);
  }
};

/// \private
template <typename T>
struct TypeHandler<NullableChecked<T>> {
 public:
  static inline Error to(NullableChecked<T> &to_type, ParseContext &context) {
    if (context.token.value_type == Type::Null) {
      to_type.null = true;
      return Error::NoError;
    }
    to_type.null = false;
    return TypeHandler<T>::to(to_type.data, context);
  }

  static inline void from(const NullableChecked<T> &opt, Token &token, Serializer &serializer) {
    if (opt.null) {
      const char nullChar[] = "null";
      token.value_type = Type::Null;
      token.value = DataRef(nullChar);
      serializer.write(token);
    } else {
      TypeHandler<T>::from(opt(), token, serializer);
    }
  }
};

/// \private
template <typename T>
struct TypeHandler<Optional<T>> {
 public:
  static inline Error to(Optional<T> &to_type, ParseContext &context) {
    return TypeHandler<T>::to(to_type.data, context);
  }

  static inline void from(const Optional<T> &opt, Token &token, Serializer &serializer) {
    TypeHandler<T>::from(opt(), token, serializer);
  }
};

/// \private
template <typename T>
struct TypeHandler<OptionalChecked<T>> {
 public:
  static inline Error to(OptionalChecked<T> &to_type, ParseContext &context) {
    to_type.assigned = true;
    return TypeHandler<T>::to(to_type.data, context);
  }

  static inline void from(const OptionalChecked<T> &opt, Token &token, Serializer &serializer) {
    if (opt.assigned)
      TypeHandler<T>::from(opt(), token, serializer);
  }
};

#ifdef JS_STD_OPTIONAL
/// \private
template <typename T>
struct TypeHandler<std::optional<T>> {
 public:
  static inline Error to(std::optional<T> &to_type, ParseContext &context) {
    to_type.emplace();
    return TypeHandler<T>::to(to_type.value(), context);
  }

  static inline void from(const std::optional<T> &opt, Token &token, Serializer &serializer) {
    if (opt.has_value())
      TypeHandler<T>::from(opt.value(), token, serializer);
  }
};
#endif

/// \private
template <typename T>
struct TypeHandler<std::shared_ptr<T>> {
 public:
  static inline Error to(std::shared_ptr<T> &to_type, ParseContext &context) {
    if (context.token.value_type != Type::Null) {
      if (!to_type)
        to_type = std::make_shared<T>();
      return TypeHandler<T>::to(*to_type.get(), context);
    }
    to_type.reset();
    return Error::NoError;
  }

  static inline void from(const std::shared_ptr<T> &unique, Token &token, Serializer &serializer) {
    if (unique) {
      TypeHandler<T>::from(*unique.get(), token, serializer);
    } else {
      const char nullChar[] = "null";
      token.value_type = Type::Null;
      token.value = DataRef(nullChar);
      serializer.write(token);
    }
  }
};

/// \private
template <typename T>
struct TypeHandler<std::unique_ptr<T>> {
 public:
  static inline Error to(std::unique_ptr<T> &to_type, ParseContext &context) {
    if (context.token.value_type != Type::Null) {
      if (!to_type)
        to_type.reset(new T());
      return TypeHandler<T>::to(*to_type.get(), context);
    }
    to_type.reset(nullptr);
    return Error::NoError;
  }

  static inline void from(const std::unique_ptr<T> &unique, Token &token, Serializer &serializer) {
    if (unique) {
      TypeHandler<T>::from(*unique.get(), token, serializer);
    } else {
      const char nullChar[] = "null";
      token.value_type = Type::Null;
      token.value = DataRef(nullChar);
      serializer.write(token);
    }
  }
};

/// \private
template <>
struct TypeHandler<bool> {
  static inline Error to(bool &to_type, ParseContext &context) {
    if (context.token.value.size == sizeof("true") - 1 &&
        memcmp("true", context.token.value.data, sizeof("true") - 1) == 0)
      to_type = true;
    else if (context.token.value.size == sizeof("false") - 1 &&
             memcmp("false", context.token.value.data, sizeof("false") - 1) == 0)
      to_type = false;
    else
      return Error::FailedToParseBoolean;

    return Error::NoError;
  }

  static inline void from(const bool &b, Token &token, Serializer &serializer) {
    const char trueChar[] = "true";
    const char falseChar[] = "false";
    token.value_type = Type::Bool;
    if (b) {
      token.value = DataRef(trueChar);
    } else {
      token.value = DataRef(falseChar);
    }
    serializer.write(token);
  }
};

#ifdef JS_STD_TIMEPOINT
/// \private
namespace Internal {
template <class T, template <class...> class Template>
struct is_specialization : std::false_type {};

template <template <class...> class Template, class... Args>
struct is_specialization<Template<Args...>, Template> : std::true_type {};
}  // namespace Internal

/// \private
template <class T>
struct TypeHandler<T, typename std::enable_if_t<Internal::is_specialization<T, std::chrono::time_point>::value>> {
  static inline Error to(T &to_type, ParseContext &context) {
    uint64_t t;
    Error err = TypeHandler<uint64_t>::to(t, context);
    if (err != Error::NoError)
      return err;

    if (t <= 1e11)  // Seconds => 10 digits, normally
      to_type = T{std::chrono::seconds{t}};
    else if (t <= 1e14)  // Milliseconds => 13 digits, normally
      to_type = T{std::chrono::milliseconds{t}};
    else if (t <= 1e17)  // Microseconds
      to_type = T{std::chrono::microseconds{t}};
    else if (t <= 1e20)  // Nanoseconds
      if constexpr (std::is_same_v<std::chrono::high_resolution_clock::time_point, T>)
        to_type = T{std::chrono::nanoseconds{t}};
      else
        return JS::Error::IllegalDataValue;
    else
      return JS::Error::IllegalDataValue;

    return JS::Error::NoError;
  }

  static inline void from(const T &val, Token &token, Serializer &serializer) {
    uint64_t t;
    if constexpr (std::is_same_v<std::chrono::high_resolution_clock::time_point, T>)
      t = std::chrono::duration_cast<std::chrono::nanoseconds>(val.time_since_epoch()).count();
    else
      t = std::chrono::duration_cast<std::chrono::microseconds>(val.time_since_epoch()).count();
    while (t % 1000 == 0 && t > (uint64_t)1e10)
      t /= 1000;
    TypeHandler<uint64_t>::from(t, token, serializer);
  }
};
#endif

/// \private
template <typename T>
struct TypeHandler<std::vector<T>> {
  static inline Error to(std::vector<T> &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart)
      return Error::ExpectedArrayStart;
    Error error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    to_type.clear();
    to_type.reserve(10);
    while (context.token.value_type != JS::Type::ArrayEnd) {
      to_type.push_back(T());
      error = TypeHandler<T>::to(to_type.back(), context);
      if (error != JS::Error::NoError)
        break;
      error = context.nextToken();
      if (error != JS::Error::NoError)
        break;
    }

    return error;
  }

  static inline void from(const std::vector<T> &vec, Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");

    for (auto &index : vec) {
      TypeHandler<T>::from(index, token, serializer);
    }

    token.name = DataRef("");

    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};

/// \private
template <>
struct TypeHandler<std::vector<bool>> {
 public:
  static inline Error to(std::vector<bool> &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart)
      return Error::ExpectedArrayStart;
    Error error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    to_type.clear();
    to_type.reserve(10);
    while (context.token.value_type != JS::Type::ArrayEnd) {
      bool toBool;
      error = TypeHandler<bool>::to(toBool, context);
      to_type.push_back(toBool);
      if (error != JS::Error::NoError)
        break;
      error = context.nextToken();
      if (error != JS::Error::NoError)
        break;
    }

    return error;
  }

  static inline void from(const std::vector<bool> &vec, Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");

    for (bool index : vec) {
      TypeHandler<bool>::from(index, token, serializer);
    }

    token.name = DataRef("");

    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};

/// \private
template <>
struct TypeHandler<SilentString> {
  static inline Error to(SilentString &to_type, ParseContext &context) {
    return TypeHandler<std::string>::to(to_type.data, context);
  }
  static inline void from(const SilentString &str, Token &token, Serializer &serializer) {
    if (str.data.size()) {
      TypeHandler<std::string>::from(str.data, token, serializer);
    }
  }
};

/// \private
template <typename T>
struct TypeHandler<SilentVector<T>> {
 public:
  static inline Error to(SilentVector<T> &to_type, ParseContext &context) {
    return TypeHandler<std::vector<T>>::to(to_type.data, context);
  }

  static inline void from(const SilentVector<T> &vec, Token &token, Serializer &serializer) {
    if (vec.data.size()) {
      TypeHandler<std::vector<T>>::from(vec.data, token, serializer);
    }
  }
};

/// \private
template <typename T>
struct TypeHandler<SilentUniquePtr<T>> {
 public:
  static inline Error to(SilentUniquePtr<T> &to_type, ParseContext &context) {
    return TypeHandler<std::unique_ptr<T>>::to(to_type.data, context);
  }

  static inline void from(const SilentUniquePtr<T> &ptr, Token &token, Serializer &serializer) {
    if (ptr.data) {
      TypeHandler<std::unique_ptr<T>>::from(ptr.data, token, serializer);
    }
  }
};

/// \private
template <>
struct TypeHandler<std::vector<Token>> {
 public:
  static inline Error to(std::vector<Token> &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart && context.token.value_type != JS::Type::ObjectStart) {
      to_type.push_back(context.token);
      return context.error;
    }
    to_type.clear();
    to_type.push_back(context.token);
    bool buffer_change = false;
    auto ref = context.tokenizer.registerNeedMoreDataCallback([&buffer_change](JS::Tokenizer &tokenizer) {
      JS_UNUSED(tokenizer);
      buffer_change = true;
    });

    size_t level = 1;
    Error error = Error::NoError;
    while (error == JS::Error::NoError && level && buffer_change == false) {
      error = context.nextToken();
      to_type.push_back(context.token);
      if (context.token.value_type == Type::ArrayStart || context.token.value_type == Type::ObjectStart)
        level++;
      else if (context.token.value_type == Type::ArrayEnd || context.token.value_type == Type::ObjectEnd)
        level--;
    }
    if (buffer_change)
      return Error::NonContigiousMemory;

    return error;
  }

  static inline void from(const std::vector<Token> &from_type, Token &token, Serializer &serializer) {
    for (auto &t : from_type) {
      token = t;
      serializer.write(token);
    }
  }
};

/// \private
template <>
struct TypeHandler<JsonTokens> {
 public:
  static inline Error to(JsonTokens &to_type, ParseContext &context) {
    return TypeHandler<std::vector<Token>>::to(to_type.data, context);
  }
  static inline void from(const JsonTokens &from, Token &token, Serializer &serializer) {
    return TypeHandler<std::vector<Token>>::from(from.data, token, serializer);
  }
};

/// \private
template <>
struct TypeHandler<JsonArrayRef> {
  static inline Error to(JsonArrayRef &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart)
      return Error::ExpectedArrayStart;

    bool buffer_change = false;
    auto ref = context.tokenizer.registerNeedMoreDataCallback([&buffer_change](JS::Tokenizer &tokenizer) {
      JS_UNUSED(tokenizer);
      buffer_change = true;
    });

    to_type.ref.data = context.token.value.data;

    size_t level = 1;
    Error error = Error::NoError;
    while (error == JS::Error::NoError && level && buffer_change == false) {
      error = context.nextToken();
      if (context.token.value_type == Type::ArrayStart)
        level++;
      else if (context.token.value_type == Type::ArrayEnd)
        level--;
    }
    if (buffer_change)
      return Error::NonContigiousMemory;

    to_type.ref.size = size_t(context.token.value.data + context.token.value.size - to_type.ref.data);

    return error;
  }

  static inline void from(const JsonArrayRef &from_type, Token &token, Serializer &serializer) {
    token.value = from_type.ref;
    token.value_type = Type::Verbatim;
    serializer.write(token);
  }
};

/// \private
template <>
struct TypeHandler<JsonArray> {
  static inline Error to(JsonArray &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart)
      return Error::ExpectedArrayStart;

    context.tokenizer.copyFromValue(context.token, to_type.data);

    size_t level = 1;
    Error error = Error::NoError;
    while (error == JS::Error::NoError && level) {
      error = context.nextToken();
      if (context.token.value_type == Type::ArrayStart)
        level++;
      else if (context.token.value_type == Type::ArrayEnd)
        level--;
    }

    if (error == JS::Error::NoError)
      context.tokenizer.copyIncludingValue(context.token, to_type.data);

    return error;
  }

  static inline void from(const JsonArray &from_type, Token &token, Serializer &serializer) {
    token.value_type = JS::Type::Verbatim;  // Need to fool the serializer to just write value as verbatim

    if (from_type.data.empty()) {
      std::string emptyArray("[]");
      token.value = DataRef(emptyArray);
      serializer.write(token);
    } else {
      token.value = DataRef(from_type.data);
      serializer.write(token);
    }
  }
};

/// \private
template <>
struct TypeHandler<JsonObjectRef> {
  static inline Error to(JsonObjectRef &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ObjectStart)
      return Error::ExpectedObjectStart;

    bool buffer_change = false;
    auto ref = context.tokenizer.registerNeedMoreDataCallback([&buffer_change](JS::Tokenizer &tokenizer) {
      JS_UNUSED(tokenizer);
      buffer_change = true;
    });

    to_type.ref.data = context.token.value.data;
    size_t level = 1;
    Error error = Error::NoError;
    while (error == JS::Error::NoError && level && buffer_change == false) {
      error = context.nextToken();
      if (context.token.value_type == Type::ObjectStart)
        level++;
      else if (context.token.value_type == Type::ObjectEnd)
        level--;
    }
    if (buffer_change)
      return Error::NonContigiousMemory;

    to_type.ref.size = size_t(context.token.value.data + context.token.value.size - to_type.ref.data);
    return error;
  }

  static inline void from(const JsonObjectRef &from_type, Token &token, Serializer &serializer) {
    token.value = from_type.ref;
    token.value_type = Type::Verbatim;
    serializer.write(token);
  }
};

/// \private
template <>
struct TypeHandler<JsonObject> {
  static inline Error to(JsonObject &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ObjectStart)
      return Error::ExpectedObjectStart;

    context.tokenizer.copyFromValue(context.token, to_type.data);

    size_t level = 1;
    Error error = Error::NoError;
    while (error == JS::Error::NoError && level) {
      error = context.nextToken();
      if (context.token.value_type == Type::ObjectStart)
        level++;
      else if (context.token.value_type == Type::ObjectEnd)
        level--;
    }

    context.tokenizer.copyIncludingValue(context.token, to_type.data);

    return error;
  }

  static inline void from(const JsonObject &from_type, Token &token, Serializer &serializer) {
    token.value_type = JS::Type::Verbatim;  // Need to fool the serializer to just write value as verbatim

    if (from_type.data.empty()) {
      std::string emptyObject("{}");
      token.value = DataRef(emptyObject);
      serializer.write(token);
    } else {
      token.value = DataRef(from_type.data);
      serializer.write(token);
    }
  }
};

/// \private
template <>
struct TypeHandler<JsonObjectOrArrayRef> {
  static inline Error to(JsonObjectOrArrayRef &to_type, ParseContext &context) {
    JS::Type openType;
    JS::Type closeType;
    if (context.token.value_type == JS::Type::ObjectStart) {
      openType = JS::Type::ObjectStart;
      closeType = JS::Type::ObjectEnd;
    } else if (context.token.value_type == JS::Type::ArrayStart) {
      openType = JS::Type::ArrayStart;
      closeType = JS::Type::ArrayEnd;
    } else {
      return Error::ExpectedObjectStart;
    }

    bool buffer_change = false;
    auto ref = context.tokenizer.registerNeedMoreDataCallback([&buffer_change](JS::Tokenizer &tokenizer) {
      JS_UNUSED(tokenizer);
      buffer_change = true;
    });

    to_type.ref.data = context.token.value.data;
    size_t level = 1;
    Error error = Error::NoError;
    while (error == JS::Error::NoError && level && buffer_change == false) {
      error = context.nextToken();
      if (context.token.value_type == openType)
        level++;
      else if (context.token.value_type == closeType)
        level--;
    }
    if (buffer_change)
      return Error::NonContigiousMemory;

    to_type.ref.size = size_t(context.token.value.data + context.token.value.size - to_type.ref.data);
    return error;
  }

  static inline void from(const JsonObjectOrArrayRef &from_type, Token &token, Serializer &serializer) {
    token.value = from_type.ref;
    token.value_type = Type::Verbatim;
    serializer.write(token);
  }
};

/// \private
template <>
struct TypeHandler<JsonObjectOrArray> {
  static inline Error to(JsonObjectOrArray &to_type, ParseContext &context) {
    JS::Type openType;
    JS::Type closeType;
    if (context.token.value_type == JS::Type::ObjectStart) {
      openType = JS::Type::ObjectStart;
      closeType = JS::Type::ObjectEnd;
    } else if (context.token.value_type == JS::Type::ArrayStart) {
      openType = JS::Type::ArrayStart;
      closeType = JS::Type::ArrayEnd;
    } else {
      return Error::ExpectedObjectStart;
    }

    context.tokenizer.copyFromValue(context.token, to_type.data);

    size_t level = 1;
    Error error = Error::NoError;
    while (error == JS::Error::NoError && level) {
      error = context.nextToken();
      if (context.token.value_type == openType)
        level++;
      else if (context.token.value_type == closeType)
        level--;
    }

    context.tokenizer.copyIncludingValue(context.token, to_type.data);

    return error;
  }

  static inline void from(const JsonObjectOrArray &from_type, Token &token, Serializer &serializer) {
    token.value_type = JS::Type::Verbatim;  // Need to fool the serializer to just write value as verbatim

    if (from_type.data.empty()) {
      std::string emptyObjectOrArray("{}");  // Use object as default
      token.value = DataRef(emptyObjectOrArray);
      serializer.write(token);
    } else {
      token.value = DataRef(from_type.data);
      serializer.write(token);
    }
  }
};

namespace Internal {
template <size_t INDEX, typename... Ts>
struct TupleTypeHandler {
  static inline Error to(JS::Tuple<Ts...> &to_type, ParseContext &context) {
    using Type = typename JS::TypeAt<sizeof...(Ts) - INDEX, Ts...>::type;
    Error error = TypeHandler<Type>::to(to_type.template get<sizeof...(Ts) - INDEX>(), context);
    if (error != JS::Error::NoError)
      return error;
    error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    return TupleTypeHandler<INDEX - 1, Ts...>::to(to_type, context);
  }

  static inline void from(const JS::Tuple<Ts...> &from_type, Token &token, Serializer &serializer) {
    using Type = typename JS::TypeAt<sizeof...(Ts) - INDEX, Ts...>::type;
    TypeHandler<Type>::from(from_type.template get<sizeof...(Ts) - INDEX>(), token, serializer);
    TupleTypeHandler<INDEX - 1, Ts...>::from(from_type, token, serializer);
  }
};

/// \private
template <typename... Ts>
struct TupleTypeHandler<0, Ts...> {
  static inline Error to(JS::Tuple<Ts...>, ParseContext &context) {
    JS_UNUSED(context);
    return Error::NoError;
  }

  static inline void from(const JS::Tuple<Ts...> &from_type, Token &token, Serializer &serializer) {
    JS_UNUSED(from_type);
    JS_UNUSED(token);
    JS_UNUSED(serializer);
  }
};
}  // namespace Internal

/// \private
template <typename... Ts>
struct TypeHandler<JS::Tuple<Ts...>> {
  static inline Error to(JS::Tuple<Ts...> &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart)
      return Error::ExpectedArrayStart;
    Error error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    error = JS::Internal::TupleTypeHandler<sizeof...(Ts), Ts...>::to(to_type, context);
    if (error != JS::Error::NoError)
      return error;
    if (context.token.value_type != JS::Type::ArrayEnd)
      return Error::ExpectedArrayEnd;
    return Error::NoError;
  }

  static inline void from(const JS::Tuple<Ts...> &from_type, Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");

    JS::Internal::TupleTypeHandler<sizeof...(Ts), Ts...>::from(from_type, token, serializer);
    token.name = DataRef("");

    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};

namespace Internal {
template <size_t INDEX, typename... Ts>
struct StdTupleTypeHandler {
  static inline Error to(std::tuple<Ts...> &to_type, ParseContext &context) {
    using Type = typename std::tuple_element<sizeof...(Ts) - INDEX, std::tuple<Ts...>>::type;
    Error error = TypeHandler<Type>::to(std::get<sizeof...(Ts) - INDEX>(to_type), context);
    if (error != JS::Error::NoError)
      return error;
    error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    return StdTupleTypeHandler<INDEX - 1, Ts...>::to(to_type, context);
  }

  static inline void from(const std::tuple<Ts...> &from_type, Token &token, Serializer &serializer) {
    using Type = typename std::tuple_element<sizeof...(Ts) - INDEX, std::tuple<Ts...>>::type;
    TypeHandler<Type>::from(std::get<sizeof...(Ts) - INDEX>(from_type), token, serializer);
    StdTupleTypeHandler<INDEX - 1, Ts...>::from(from_type, token, serializer);
  }
};

/// \private
template <typename... Ts>
struct StdTupleTypeHandler<0, Ts...> {
  static inline Error to(std::tuple<Ts...> &, ParseContext &context) {
    JS_UNUSED(context);
    return Error::NoError;
  }

  static inline void from(const std::tuple<Ts...> &from_type, Token &token, Serializer &serializer) {
    JS_UNUSED(from_type);
    JS_UNUSED(token);
    JS_UNUSED(serializer);
  }
};
}  // namespace Internal
/// \private
template <typename... Ts>
struct TypeHandler<std::tuple<Ts...>> {
  static inline Error to(std::tuple<Ts...> &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart)
      return Error::ExpectedArrayStart;
    Error error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    error = JS::Internal::StdTupleTypeHandler<sizeof...(Ts), Ts...>::to(to_type, context);
    if (error != JS::Error::NoError)
      return error;
    if (context.token.value_type != JS::Type::ArrayEnd)
      return Error::ExpectedArrayEnd;
    return Error::NoError;
  }

  static inline void from(const std::tuple<Ts...> &from_type, Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");

    JS::Internal::StdTupleTypeHandler<sizeof...(Ts), Ts...>::from(from_type, token, serializer);
    token.name = DataRef("");

    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};

template <typename T>
struct OneOrMany {
  std::vector<T> data;
};

template <typename T>
struct TypeHandler<OneOrMany<T>> {
 public:
  static inline Error to(OneOrMany<T> &to_type, ParseContext &context) {
    if (context.token.value_type == Type::ArrayStart) {
      context.error = TypeHandler<std::vector<T>>::to(to_type.data, context);
    } else {
      to_type.data.push_back(T());
      context.error = TypeHandler<T>::to(to_type.data.back(), context);
    }
    return context.error;
  }
  static void from(const OneOrMany<T> &from, Token &token, Serializer &serializer) {
    if (from.data.empty())
      return;
    if (from.data.size() > 1) {
      TypeHandler<std::vector<T>>::from(from.data, token, serializer);
    } else {
      TypeHandler<T>::from(from.data.front(), token, serializer);
    }
  }
};

template <typename T, size_t N>
struct TypeHandler<T[N]> {
 public:
  static inline Error to(T (&to_type)[N], ParseContext &context) {
    if (context.token.value_type != Type::ArrayStart)
      return JS::Error::ExpectedArrayStart;

    context.nextToken();
    for (size_t i = 0; i < N; i++) {
      if (context.error != JS::Error::NoError)
        return context.error;
      context.error = TypeHandler<T>::to(to_type[i], context);
      if (context.error != JS::Error::NoError)
        return context.error;

      context.nextToken();
    }

    if (context.token.value_type != Type::ArrayEnd)
      return JS::Error::ExpectedArrayEnd;
    return context.error;
  }
  static void from(const T (&from)[N], Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");
    for (size_t i = 0; i < N; i++)
      TypeHandler<T>::from(from[i], token, serializer);

    token.name = DataRef("");
    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};

template <typename Key, typename Value, typename Map>
struct TypeHandlerMap {
  static inline Error to(Map &to_type, ParseContext &context) {
    if (context.token.value_type != Type::ObjectStart) {
      return JS::Error::ExpectedObjectStart;
    }

    Error error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    while (context.token.value_type != Type::ObjectEnd) {
      Key key(context.token.name.data, context.token.name.size);
      Value v;
      error = TypeHandler<Value>::to(v, context);
      to_type[std::move(key)] = std::move(v);
      if (error != JS::Error::NoError)
        return error;
      error = context.nextToken();
    }

    return error;
  }

  static void from(const Map &from, Token &token, Serializer &serializer) {
    token.value_type = Type::ObjectStart;
    token.value = DataRef("{");
    serializer.write(token);
    for (auto it = from.begin(); it != from.end(); ++it) {
      token.name = DataRef(it->first);
      token.name_type = Type::String;
      TypeHandler<Value>::from(it->second, token, serializer);
    }
    token.name.size = 0;
    token.name.data = "";
    token.name_type = Type::String;
    token.value_type = Type::ObjectEnd;
    token.value = DataRef("}");
    serializer.write(token);
  }
};

#ifdef JS_STD_UNORDERED_MAP
template <typename Key, typename Value>
struct TypeHandler<std::unordered_map<Key, Value>> : TypeHandlerMap<Key, Value, std::unordered_map<Key, Value>> {
};

#endif

namespace Internal {
inline bool compareDataRefWithString(const DataRef &a, const std::string &b) {
  return a.size == b.size() && memcmp(a.data, b.data(), a.size) == 0;
}
}  // namespace Internal
struct Map {
  struct It {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = int;
    using value_type = Token;
    using pointer = Token *;
    using reference = Token &;
    const Map &map;
    uint32_t index = 0;
    uint32_t next_meta = 0;
    uint32_t next_complex = 0;

    It(const Map &map)
        : map(map) {
    }
    It(const It &other)
        : map(other.map), index(other.index), next_meta(other.next_meta), next_complex(other.next_complex) {
    }
    inline const Token &operator*() {
      return map.tokens.data[index];
    }

    inline const Token *operator->() {
      return &map.tokens.data[index];
    }

    inline It &operator++() {
      if (index == next_complex) {
        index += map.meta[next_meta].size;
        next_meta += map.meta[next_meta].skip;
        next_complex = next_meta < uint32_t(map.meta.size()) ? uint32_t(map.meta[next_meta].position)
                                                             : uint32_t(map.tokens.data.size());
      } else {
        index++;
      }
      return *this;
    }
    inline bool operator==(const It &other) const {
      return index == other.index;
    }
    inline bool operator!=(const It &other) const {
      return index != other.index;
    }

    inline void operator=(const It &other) {
      assert(&map == &other.map);
      index = other.index;
      next_meta = other.next_meta;
      next_complex = other.next_complex;
    }
  };

  JS::JsonTokens tokens;
  std::vector<JsonMeta> meta;
  std::vector<std::pair<int, std::string>> json_data;

  inline It begin() const {
    It b(*this);
    b.index = 1;
    b.next_meta = 1;
    b.next_complex =
        b.next_meta < uint32_t(meta.size()) ? uint32_t(meta[b.next_meta].position) : uint32_t(tokens.data.size());
    return b;
  }

  inline It end() const {
    It e(*this);
    e.index = uint32_t(tokens.data.size());
    e.next_meta = 0;
    e.next_complex = 0;
    return e;
  }

  inline It find(const std::string &name) const {
    return std::find_if(begin(), end(),
                        [&name](const Token &token) { return Internal::compareDataRefWithString(token.name, name); });
  }

  template <typename T>
  JS::Error castToType(JS::ParseContext &parseContext, T &to) const {
    parseContext.tokenizer.resetData(&tokens.data, 0);
    parseContext.nextToken();
    return JS::TypeHandler<T>::to(to, parseContext);
  }

  template <typename T>
  JS::Error castToType(const It &iterator, JS::ParseContext &parseContext, T &to) const {
    assert(iterator.index < tokens.data.size());
    parseContext.tokenizer.resetData(&tokens.data, iterator.index);
    parseContext.nextToken();
    return JS::TypeHandler<T>::to(to, parseContext);
  }

  template <typename T>
  JS::Error castToType(const std::string &name, JS::ParseContext &parseContext, T &to) const {
    if (tokens.data.empty() || tokens.data.front().value_type != JS::Type::ObjectStart) {
      parseContext.error = JS::Error::ExpectedObjectStart;
      return parseContext.error;
    }

    It it = find(name);
    if (it != end())
      return castToType(it, parseContext, to);
    parseContext.error = JS::Error::KeyNotFound;
    return parseContext.error;
  }

  template <typename T>
  T castTo(JS::ParseContext &parseContext) const {
    T t = {};
    castToType<T>(parseContext, t);
    return t;
  }

  template <typename T>
  T castTo(const std::string &name, JS::ParseContext &parseContext) const {
    T t = {};
    castToType<T>(name, parseContext, t);
    return t;
  }

  template <typename T>
  JS::Error setValue(JS::ParseContext &parseContext, const T &value) {
    static_assert(sizeof(JS::Internal::HasJsonStructBase<T>::template test_in_base<T>(nullptr)) ==
                      sizeof(typename JS::Internal::HasJsonStructBase<T>::yes),
                  "Not a Json Object type\n");
    std::string obj = JS::serializeStruct(value);
    parseContext.tokenizer.resetData(obj.data(), obj.size(), 0);
    tokens.data.clear();
    meta.clear();
    json_data.clear();
    auto error = parseContext.parseTo(tokens);
    if (error == JS::Error::NoError)
      assert(tokens.data.size() && tokens.data[0].value_type == JS::Type::ObjectStart);

    meta = metaForTokens(tokens);
    json_data.emplace_back(0, std::move(obj));
    return parseContext.error;
  }

  template <typename T>
  JS::Error setValue(const std::string &name, JS::ParseContext &parseContext, const T &value) {
    (void)parseContext;
    if (tokens.data.empty()) {
      tokens.data.reserve(10);
      meta.reserve(10);
      JS::Token token;
      token.value_type = JS::Type::ObjectStart;
      token.value = JS::DataRef("{");
      tokens.data.push_back(token);
      token.value_type = JS::Type::ObjectEnd;
      token.value = JS::DataRef("}");
      tokens.data.push_back(token);
      meta = JS::metaForTokens(tokens);
    }

    auto it = find(name);
    if (it != end()) {
      meta[0].children--;
      int tokens_removed = 0;
      if (it.index == it.next_complex) {
        auto theMeta = meta[it.next_meta];
        tokens_removed = theMeta.size;
        meta[0].complex_children--;
        meta[0].size -= theMeta.size;
        meta[0].skip -= theMeta.skip;
        auto start_token = tokens.data.begin() + it.index;
        tokens.data.erase(start_token, start_token + theMeta.size);
        int to_adjust_index = it.next_meta;
        auto start_meta = meta.begin() + it.next_meta;
        meta.erase(start_meta, start_meta + theMeta.skip);
        for (int i = to_adjust_index; i < int(meta.size()); i++) {
          meta[i].position -= theMeta.size;
        }
      } else {
        meta[0].size--;
        tokens.data.erase(tokens.data.begin() + it.index);
        tokens_removed = 1;
      }
      {
        int index_to_remove = -1;
        for (int i = 0; i < int(json_data.size()); i++) {
          if (uint32_t(json_data[i].first) == it.index)
            index_to_remove = i;
          else if (uint32_t(json_data[i].first) > it.index)
            json_data[i].first -= tokens_removed;
        }
        if (index_to_remove >= 0) {
          json_data.erase(json_data.begin() + index_to_remove);
        }
      }
    }
    static const char objectStart[] = "{";
    static const char objectEnd[] = "}";
    std::string out;
    JS::SerializerContext serializeContext(out);
    serializeContext.serializer.setOptions(SerializerOptions(JS::SerializerOptions::Compact));
    JS::Token token;
    token.value_type = Type::ObjectStart;
    token.value = DataRef(objectStart);
    serializeContext.serializer.write(token);

    token.name = DataRef(name);
    token.name_type = Type::String;
    JS::TypeHandler<T>::from(value, token, serializeContext.serializer);

    token.name = DataRef();
    token.value_type = Type::ObjectEnd;
    token.value = DataRef(objectEnd);
    serializeContext.serializer.write(token);

    serializeContext.flush();
    JS::JsonTokens new_tokens;
    JS::ParseContext pc(out.c_str(), out.size(), new_tokens);
    auto new_meta = metaForTokens(new_tokens);

    json_data.emplace_back(int(tokens.data.size() - 1), std::move(out));
    int old_tokens_size = int(tokens.data.size());
    tokens.data.insert(tokens.data.end() - 1, new_tokens.data.begin() + 1, new_tokens.data.end() - 1);
    meta[0].children++;
    if (new_meta[0].complex_children) {
      meta[0].complex_children++;
      meta[0].size += new_meta[1].size;
      meta[0].skip += new_meta[1].skip;
      int old_meta_size = int(meta.size());
      meta.insert(meta.end(), new_meta.begin() + 1, new_meta.end());
      for (int new_meta_i = old_meta_size; new_meta_i < int(meta.size()); new_meta_i++) {
        meta[new_meta_i].position +=
            old_tokens_size - 1 - 1;  // position contains an extra and old_tokens_size has another extra
      }
    } else {
      meta[0].size++;
    }

    return JS::Error::NoError;
  }
};

template <>
struct TypeHandler<Map> {
  static inline Error to(Map &to_type, ParseContext &context) {
    Error error = TypeHandler<JS::JsonTokens>::to(to_type.tokens, context);
    if (error == Error::NoError) {
      to_type.meta = metaForTokens(to_type.tokens);
    }

    return error;
  }

  static inline void from(const Map &from_type, Token &, Serializer &serializer) {
    for (auto &token : from_type.tokens.data) {
      serializer.write(token);
    }
  }
};

template <typename T, size_t COUNT>
struct ArrayVariableContent {
  T data[COUNT];
  size_t size = 0;
};

template <typename T, size_t COUNT>
struct TypeHandler<ArrayVariableContent<T, COUNT>> {
  static inline Error to(ArrayVariableContent<T, COUNT> &to_type, ParseContext &context) {
    if (context.token.value_type != Type::ArrayStart)
      return JS::Error::ExpectedArrayStart;

    context.nextToken();
    for (size_t i = 0; i < COUNT; i++) {
      if (context.error != JS::Error::NoError)
        return context.error;
      if (context.token.value_type == Type::ArrayEnd) {
        to_type.size = i;
        break;
      }
      context.error = TypeHandler<T>::to(to_type.data[i], context);
      if (context.error != JS::Error::NoError)
        return context.error;

      context.nextToken();
    }

    if (context.token.value_type != Type::ArrayEnd)
      return JS::Error::ExpectedArrayEnd;
    return context.error;
  }

  static inline void from(const ArrayVariableContent<T, COUNT> &from_type, Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");
    for (size_t i = 0; i < from_type.size; i++)
      TypeHandler<T>::from(from_type.data[i], token, serializer);

    token.name = DataRef("");
    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};

template <typename T, typename Set>
struct TypeHandlerSet {
  static inline Error to(Set &to_type, ParseContext &context) {
    if (context.token.value_type != JS::Type::ArrayStart)
      return Error::ExpectedArrayStart;
    Error error = context.nextToken();
    if (error != JS::Error::NoError)
      return error;
    to_type.clear();
    while (context.token.value_type != JS::Type::ArrayEnd) {
      T t;
      error = TypeHandler<T>::to(t, context);
      if (error != JS::Error::NoError)
        break;
      auto insert_ret = to_type.insert(std::move(t));
      if (!insert_ret.second)
        return JS::Error::DuplicateInSet;

      error = context.nextToken();
      if (error != JS::Error::NoError)
        break;
    }

    return error;
  }

  static inline void from(const Set &set, Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");

    for (auto &index : set) {
      TypeHandler<T>::from(index, token, serializer);
    }

    token.name = DataRef("");

    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};
}  // namespace JS
#endif  // JSON_STRUCT_H

#if defined(JS_STL_MAP) && !defined(JS_STL_MAP_INCLUDE)
#define JS_STL_MAP_INCLUDE
#include <map>
namespace JS {
template <typename Key, typename Value>
struct TypeHandler<std::map<Key, Value>> : TypeHandlerMap<Key, Value, std::map<Key, Value>> {
};
}  // namespace JS
#endif

#if defined(JS_STL_SET) && !defined(JS_STL_SET_INCLUDE)
#define JS_STL_SET_INCLUDE
#include <set>
namespace JS {
template <typename Key>
struct TypeHandler<std::set<Key>> : TypeHandlerSet<Key, std::set<Key>> {
};
}  // namespace JS
#endif

#if defined(JS_STL_UNORDERED_SET) && !defined(JS_STL_UNORDERED_SET_INCLUDE)
#define JS_STL_UNORDERED_SET_INCLUDE
#include <unordered_set>
namespace JS {
template <typename Key>
struct TypeHandler<std::unordered_set<Key>> : TypeHandlerSet<Key, std::unordered_set<Key>> {
};
}  // namespace JS
#endif

#if defined(JS_STL_ARRAY) && !defined(JS_STL_ARRAY_INCLUDE)
#define JS_STL_ARRAY_INCLUDE
#include <array>
namespace JS {
template <typename T, size_t N>
struct TypeHandler<std::array<T, N>> {
 public:
  static inline Error to(std::array<T, N> &to_type, ParseContext &context) {
    if (context.token.value_type != Type::ArrayStart)
      return JS::Error::ExpectedArrayStart;

    context.nextToken();
    for (size_t i = 0; i < N; i++) {
      if (context.error != JS::Error::NoError)
        return context.error;
      context.error = TypeHandler<T>::to(to_type[i], context);
      if (context.error != JS::Error::NoError)
        return context.error;

      context.nextToken();
    }

    if (context.token.value_type != Type::ArrayEnd)
      return JS::Error::ExpectedArrayEnd;
    return context.error;
  }
  static void from(const std::array<T, N> &from, Token &token, Serializer &serializer) {
    token.value_type = Type::ArrayStart;
    token.value = DataRef("[");
    serializer.write(token);

    token.name = DataRef("");
    for (size_t i = 0; i < N; i++)
      TypeHandler<T>::from(from[i], token, serializer);

    token.name = DataRef("");
    token.value_type = Type::ArrayEnd;
    token.value = DataRef("]");
    serializer.write(token);
  }
};
}  // namespace JS
#endif

#if defined(JS_INT_128) && !defined(JS_INT_128_INCLUDE)
#define JS_INT_128_INCLUDE 1
// Compiler support check
#if defined(__SIZEOF_INT128__) && !defined(JS_NO_INT128_TYPEDEF)
namespace JS {
__extension__ using js_int128_t = __int128;
__extension__ using js_uint128_t = unsigned __int128;
}  // namespace JS
#endif

namespace JS {
/// \private
template <>
struct TypeHandler<js_int128_t> : TypeHandlerIntType<js_int128_t> {
};

/// \private
template <>
struct TypeHandler<js_uint128_t> : TypeHandlerIntType<js_uint128_t> {
};
}  // namespace JS
#endif
