/*
 * Copyright © 2018 Øystein Myrmo

 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission. The copyright holders make no representations
 * about the suitability of this software for any purpose. It is provided "as
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

/*! \mainpage json_struct_diff
 *
 * json_struct_diff is an extension to json_struct to efficiently diff json by
 * using json_struct itself.
 */

#ifndef JSON_STRUCT_DIFF_H
#define JSON_STRUCT_DIFF_H

#include "json_struct.h"

namespace JS {

enum DiffFlags : unsigned char {
  None,
  FuzzyFloatComparison
};

enum DiffType : int {
  NoDiff,             // Type and value are equal
  ValueDiff,          // Value has changed
  TypeDiff,           // Type has changed
  NewMember,          // New member in JSON object
  NewArrayItem,       // New item in JSON array
  MissingMembers,     // JSON object is missing one or more members
  MissingArrayItems,  // JSON array is missing one or more items
  RootItemDiff,       // The root items are different
  ErroneousRootItem   // The root item is not Type::ObjectStart or Type::ArrayStart
};

enum DiffError : unsigned char {
  NoError,
  NoTokens,
  EmptyString
};

struct DiffOptions {
  DiffOptions() {}
  DiffOptions(DiffFlags flags, double fuzzyEpsilon)
      : flags(flags), fuzzyEpsilon(fuzzyEpsilon) {}
  DiffFlags flags = DiffFlags::FuzzyFloatComparison;
  double fuzzyEpsilon = 1e-6;  // Used if (flags | FuzzyFloatComparison == true)
};

namespace Internal {
namespace Diff {
inline bool isComplexValue(const Token &token) {
  return (token.value_type == Type::ObjectStart) || (token.value_type == Type::ArrayStart);
}

struct MissingTokens {
  Token token;
  std::vector<Token> missingTokens;

  bool operator==(const Token &t) const {
    return (token.name.data == t.name.data) && (token.value.data == t.value.data);
  }
};
}  // namespace Diff
}  // namespace Internal

struct DiffTokens {
  DiffTokens() {}

  explicit DiffTokens(const char *json, size_t size) {
    reset(json, size);
  }

  void reset(const char *json, size_t size) {
    clear();
    tokens.data.reserve(50);
    generateTokens(json, size);
    diffs.resize(tokens.data.size(), DiffType::NoDiff);
    meta = metaForTokens(tokens);
  }

  void invalidate() {
    missingMembers.clear();
    diffs.clear();
    error = DiffError::NoError;
    diff_count = 0;
    diffs.resize(tokens.data.size(), DiffType::NoDiff);
  }

  void clear() {
    tokens.data.clear();
    missingMembers.clear();
    diffs.clear();
    meta.clear();
    error = DiffError::NoError;
    diff_count = 0;
  }

  void generateTokens(const char *json, size_t size) {
    if (size == 0) {
      error = DiffError::EmptyString;
      return;
    }

    Tokenizer tokenizer;
    tokenizer.addData(json, size);
    Token token;
    Error e = Error::NoError;
    while (e == Error::NoError) {
      e = tokenizer.nextToken(token);
      if (e == Error::NoError)
        tokens.data.emplace_back(token);
    }
    if (e == Error::NeedMoreData)
      error = DiffError::NoError;
    if (tokens.data.size() == 0)
      error = DiffError::NoTokens;
  }

  void skip(size_t *pos) const {
    assert(*pos < size());
    if (Internal::Diff::isComplexValue(tokens.data[*pos])) {
      for (const auto &m : meta) {
        if (m.position == *pos) {
          *pos += m.size;
          return;
        }
      }
    } else {
      (*pos)++;  // Skip strings, numbers, bools, nulls
    }
  }

  unsigned int childCount(size_t pos) const {
    assert(pos < size());
    assert(Internal::Diff::isComplexValue(tokens.data[pos]));
    size_t metaPos;
    if (getMetaPos(pos, &metaPos))
      return meta[metaPos].children;
    else
      return 0;
  }

  void set(size_t pos, DiffType diff) {
    assert(pos < tokens.data.size());
    diffs[pos] = diff;
    if (diff != DiffType::NoDiff)
      diff_count++;
  }

  void setStateForEntireToken(const size_t startPos, DiffType diffType) {
    assert(startPos < size());
    set(startPos, diffType);
    const Token &token = tokens.data[startPos];

    if (token.value_type == Type::ObjectStart || token.value_type == Type::ArrayStart) {
      size_t pos = startPos + 1;
      while (true) {
        const Token &currentToken = tokens.data[pos];
        if (currentToken.value_type == Type::ObjectStart || currentToken.value_type == Type::ArrayStart) {
          setStateForEntireToken(pos, diffType);
          skip(&pos);
        } else if (currentToken.value_type == Type::ObjectEnd || currentToken.value_type == Type::ArrayEnd) {
          set(pos, diffType);
          break;
        } else {
          set(pos, diffType);
          skip(&pos);
        }
      }
    }
  }

  bool getMetaPos(size_t pos, size_t *outPos) const {
    bool set = false;
    for (size_t i = 0; i < meta.size(); i++) {
      if (meta[i].position == pos) {
        *outPos = i;
        set = true;
        break;
      }
    }
    return set;
  }

  void addMissingMembers(const size_t startPos, const DiffTokens &baseTokens, const size_t basePos) {
    assert(startPos < size());
    assert(tokens.data[startPos].value_type == JS::Type::ObjectStart);
    const Token &token = baseTokens.tokens.data[basePos];
    addMissingToken(startPos, token);

    if (token.value_type == Type::ObjectStart || token.value_type == Type::ArrayStart) {
      size_t pos = basePos + 1;
      while (true) {
        const Token &missingToken = baseTokens.tokens.data[pos];
        if (missingToken.value_type == Type::ObjectStart || missingToken.value_type == Type::ArrayStart) {
          addMissingMembers(startPos, baseTokens, pos);
          baseTokens.skip(&pos);
        } else if (missingToken.value_type == Type::ObjectEnd || missingToken.value_type == Type::ArrayEnd) {
          addMissingToken(startPos, missingToken);
          break;
        } else {
          addMissingToken(startPos, missingToken);
          baseTokens.skip(&pos);
        }
      }
    }
  }

  void addMissingToken(const size_t pos, const Token &missingMember) {
    assert(pos < tokens.data.size());
    const Token &objectMissingMember = tokens.data[pos];
    assert(objectMissingMember.value_type == Type::ObjectStart);

    std::vector<Token> *existing = nullptr;
    for (auto &m : missingMembers) {
      if (m == objectMissingMember) {
        existing = &m.missingTokens;
        break;
      }
    }

    if (existing) {
      existing->emplace_back(missingMember);
    } else {
      Internal::Diff::MissingTokens m;
      m.token = objectMissingMember;
      m.missingTokens.reserve(10);
      m.missingTokens.emplace_back(missingMember);
      missingMembers.emplace_back(m);
    }

    set(pos, DiffType::MissingMembers);
  }

  void addMissingArrayItems(const size_t startPos, const DiffTokens &baseTokens, const size_t basePos) {
    assert(startPos < size());
    assert(tokens.data[startPos].value_type == JS::Type::ArrayStart);
    const Token &token = baseTokens.tokens.data[basePos];
    addMissingArrayToken(startPos, token);

    if (token.value_type == Type::ObjectStart || token.value_type == Type::ArrayStart) {
      size_t pos = basePos + 1;
      while (true) {
        const Token &missingToken = baseTokens.tokens.data[pos];
        if (missingToken.value_type == Type::ObjectStart || missingToken.value_type == Type::ArrayStart) {
          addMissingArrayItems(startPos, baseTokens, pos);
          baseTokens.skip(&pos);
        } else if (missingToken.value_type == Type::ObjectEnd || missingToken.value_type == Type::ArrayEnd) {
          addMissingArrayToken(startPos, missingToken);
          break;
        } else {
          addMissingArrayToken(startPos, missingToken);
          baseTokens.skip(&pos);
        }
      }
    }
  }

  void addMissingArrayToken(const size_t pos, const Token &missingItem) {
    assert(pos < tokens.data.size());
    const Token &arrayMissingItem = tokens.data[pos];
    assert(arrayMissingItem.value_type == Type::ArrayStart);

    std::vector<Token> *existing = nullptr;
    for (auto &m : missingArrayItems) {
      if (m == arrayMissingItem) {
        existing = &m.missingTokens;
        break;
      }
    }

    if (existing) {
      existing->emplace_back(missingItem);
    } else {
      Internal::Diff::MissingTokens m;
      m.token = arrayMissingItem;
      m.missingTokens.reserve(10);
      m.missingTokens.emplace_back(missingItem);
      missingArrayItems.emplace_back(m);
    }

    set(pos, DiffType::MissingArrayItems);
  }

  size_t size() const {
    assert(tokens.data.size() == diffs.size());
    return tokens.data.size();
  }

  const std::vector<Token> *getMissingTokens(const Token &token, const std::vector<Internal::Diff::MissingTokens> &missingTokens) const {
    const std::vector<Token> *missing = nullptr;
    for (auto const &m : missingTokens) {
      if (m == token) {
        missing = &m.missingTokens;
        break;
      }
    }
    return missing;
  }

  const std::vector<Token> *getMissingMembers(const Token &token) const {
    return getMissingTokens(token, missingMembers);
  }

  const std::vector<Token> *getMissingArrayItems(const Token &token) const {
    return getMissingTokens(token, missingArrayItems);
  }

  JsonTokens tokens;
  std::vector<Internal::Diff::MissingTokens> missingMembers;
  std::vector<Internal::Diff::MissingTokens> missingArrayItems;
  std::vector<DiffType> diffs;  // Equal length to tokens.data array, having diffs in the same order
  std::vector<JsonMeta> meta;
  DiffError error;
  size_t diff_count = 0;
};

namespace Internal {
namespace Diff {
void diffObjects(const DiffTokens &base, const size_t basePos, DiffTokens &diff, const size_t diffPos, const DiffOptions &options);
void diffArrays(const DiffTokens &base, const size_t basePos, DiffTokens &diff, const size_t diffPos, const DiffOptions &options);

inline bool isSameMember(const Token &base, const Token &diff) {
  return (base.name_type == diff.name_type) && (base.name.size == diff.name.size) && (strncmp(base.name.data, diff.name.data, base.name.size) == 0);
}

inline bool isSameValueType(const Token &base, const Token &diff) {
  return base.value_type == diff.value_type;
}

inline bool stringValuesEqual(const DataRef &base, const DataRef &diff) {
  return (base.size == diff.size) && (strncmp(base.data, diff.data, base.size) == 0);
}

inline bool fuzzyEquals(double d1, double d2, double epsilon) {
  double diff = d2 - d1;
  return diff > 0 ? (diff <= epsilon) : (-diff <= epsilon);
}

inline void diffStringValues(const Token &baseToken, const Token &diffToken, DiffTokens &diff, const size_t diffPos) {
  if (stringValuesEqual(baseToken.value, diffToken.value))
    diff.set(diffPos, DiffType::NoDiff);
  else
    diff.set(diffPos, DiffType::ValueDiff);
}

inline void diffNumberValues(const Token &baseToken, const Token &diffToken, DiffTokens &diff, const size_t diffPos, const DiffOptions &options) {
  double baseValue = std::stod(std::string(baseToken.value.data, baseToken.value.size));
  double diffValue = std::stod(std::string(diffToken.value.data, diffToken.value.size));

  if (options.flags & DiffFlags::FuzzyFloatComparison) {
    if (fuzzyEquals(baseValue, diffValue, options.fuzzyEpsilon))
      diff.set(diffPos, DiffType::NoDiff);
    else
      diff.set(diffPos, DiffType::ValueDiff);
  } else {
    if (baseValue == diffValue)
      diff.set(diffPos, DiffType::NoDiff);
    else
      diff.set(diffPos, DiffType::ValueDiff);
  }
}

inline void diffBooleanValues(const Token &baseToken, const Token &diffToken, DiffTokens &diff, const size_t diffPos) {
  bool baseBool = false;
  bool diffBool = false;
  ParseContext dummyContext;

  dummyContext.token = baseToken;
  TypeHandler<bool>::to(baseBool, dummyContext);
  assert(dummyContext.error == JS::Error::NoError);

  dummyContext.token = diffToken;
  TypeHandler<bool>::to(diffBool, dummyContext);
  assert(dummyContext.error == JS::Error::NoError);

  if (baseBool ^ diffBool)
    diff.set(diffPos, DiffType::ValueDiff);
  else
    diff.set(diffPos, DiffType::NoDiff);
}

inline void diffNullValues(const Token &baseToken, const Token &diffToken, DiffTokens &diff, const size_t diffPos) {
  if ((baseToken.value_type == Type::Null) && (diffToken.value_type == Type::Null))
    diff.set(diffPos, DiffType::NoDiff);
  else
    diff.set(diffPos, DiffType::TypeDiff);
}

inline void diffObjectMember(const DiffTokens &base, const size_t basePos, DiffTokens &diff, const size_t diffPos, const DiffOptions &options) {
  auto const &baseToken = base.tokens.data[basePos];
  auto const &diffToken = diff.tokens.data[diffPos];
  assert(isSameMember(baseToken, diffToken));
  assert(isSameValueType(baseToken, diffToken));

  if (baseToken.value_type == Type::String)
    diffStringValues(baseToken, diffToken, diff, diffPos);
  else if (baseToken.value_type == Type::Number)
    diffNumberValues(baseToken, diffToken, diff, diffPos, options);
  else if (baseToken.value_type == Type::Bool)
    diffBooleanValues(baseToken, diffToken, diff, diffPos);
  else if (baseToken.value_type == Type::Null)
    diffNullValues(baseToken, diffToken, diff, diffPos);
  else if (baseToken.value_type == Type::ObjectStart)
    diffObjects(base, basePos, diff, diffPos, options);
  else if (baseToken.value_type == Type::ArrayStart)
    diffArrays(base, basePos, diff, diffPos, options);
}

inline void setStateForEntireToken(DiffTokens &diffTokens, const size_t startPos, DiffType diffType) {
  assert(startPos < diffTokens.size());
  diffTokens.set(startPos, diffType);
  const Token &token = diffTokens.tokens.data[startPos];

  if (token.value_type == Type::ObjectStart || token.value_type == Type::ArrayStart) {
    size_t pos = startPos + 1;
    while (true) {
      const Token &currentToken = diffTokens.tokens.data[pos];
      if (currentToken.value_type == Type::ObjectStart || currentToken.value_type == Type::ArrayStart) {
        setStateForEntireToken(diffTokens, pos, diffType);
        diffTokens.skip(&pos);
      } else if (currentToken.value_type == Type::ObjectEnd || currentToken.value_type == Type::ArrayEnd) {
        diffTokens.set(pos, diffType);
        break;
      } else {
        diffTokens.set(pos, diffType);
        diffTokens.skip(&pos);
      }
    }
  }
}

inline void diffObjects(const DiffTokens &base, const size_t basePos, DiffTokens &diff, const size_t diffPos, const DiffOptions &options) {
  assert(base.tokens.data[basePos].value_type == Type::ObjectStart);
  assert(diff.tokens.data[diffPos].value_type == Type::ObjectStart);
  assert(basePos < base.tokens.data.size());
  assert(diffPos < diff.tokens.data.size());

  size_t bChildCount = base.childCount(basePos);
  size_t dChildCount = diff.childCount(diffPos);
  if (bChildCount == 0 && dChildCount == 0)
    return;

  size_t bPos = basePos + 1;
  size_t dPos = diffPos + 1;

  // Diff members that exist in both objects and find members that are in base but not in diff.
  while (true) {
    const Token &baseToken = base.tokens.data[bPos];
    if (baseToken.value_type == Type::ObjectEnd)
      break;

    dPos = diffPos + 1;
    while (true) {
      const Token &diffToken = diff.tokens.data[dPos];
      if (diffToken.value_type == Type::ObjectEnd) {
        // Did not find baseToken in diff: missing member.
        diff.addMissingMembers(diffPos, base, bPos);
        base.skip(&bPos);
        diff.skip(&dPos);
        break;
      }

      if (isSameMember(baseToken, diffToken)) {
        if (isSameValueType(baseToken, diffToken))
          diffObjectMember(base, bPos, diff, dPos, options);
        else
          setStateForEntireToken(diff, dPos, DiffType::TypeDiff);
        base.skip(&bPos);
        break;
      }

      diff.skip(&dPos);
    }
  }

  // Find members that are in diff but not in base.
  bPos = basePos + 1;
  dPos = diffPos + 1;
  while (true) {
    const Token &diffToken = diff.tokens.data[dPos];
    if (diffToken.value_type == Type::ObjectEnd)
      break;

    bPos = basePos + 1;
    while (true) {
      const Token &baseToken = base.tokens.data[bPos];
      if (baseToken.value_type == Type::ObjectEnd) {
        // Did not find diffToken in base: new member.
        setStateForEntireToken(diff, dPos, DiffType::NewMember);
        diff.skip(&dPos);
        break;
      }

      if (isSameMember(diffToken, baseToken)) {
        diff.skip(&dPos);
        break;
      }

      base.skip(&bPos);
    }
  }
}

inline void diffArrays(const DiffTokens &base, const size_t basePos, DiffTokens &diff, const size_t diffPos, const DiffOptions &options) {
  assert(base.tokens.data[basePos].value_type == Type::ArrayStart);
  assert(diff.tokens.data[diffPos].value_type == Type::ArrayStart);

  size_t bChildCount = base.childCount(basePos);
  size_t dChildCount = diff.childCount(diffPos);
  if (bChildCount == 0 && dChildCount == 0)
    return;

  size_t bPos = basePos + 1;
  size_t dPos = diffPos + 1;
  bool arrayDiffDone = false;

  while (!arrayDiffDone) {
    const Token &baseToken = base.tokens.data[bPos];
    const Token &diffToken = diff.tokens.data[dPos];
    if (baseToken.value_type == diffToken.value_type) {
      switch (baseToken.value_type) {
        case Type::ArrayEnd:
          arrayDiffDone = true;
          continue;
        case Type::ObjectStart:
          diffObjects(base, bPos, diff, dPos, options);
          break;
        case Type::ArrayStart:
          diffArrays(base, bPos, diff, dPos, options);
          break;
        case Type::String:
          diffStringValues(baseToken, diffToken, diff, dPos);
          break;
        case Type::Number:
          diffNumberValues(baseToken, diffToken, diff, dPos, options);
          break;
        case Type::Bool:
          diffBooleanValues(baseToken, diffToken, diff, dPos);
          break;
        case Type::Null:
          diffNullValues(baseToken, diffToken, diff, dPos);
          break;
        default:
          assert(false);  // Not implemented, this is an error!
          break;
      }

      base.skip(&bPos);
      diff.skip(&dPos);
    } else {
      // New items in array.
      if (baseToken.value_type == Type::ArrayEnd) {
        int arrayEndCount = 0;
        while (arrayEndCount >= 0) {
          diff.set(dPos++, DiffType::NewArrayItem);
          const Token &dToken = diff.tokens.data[dPos];
          if (dToken.value_type == Type::ArrayStart)
            arrayEndCount++;
          else if (dToken.value_type == Type::ArrayEnd)
            arrayEndCount--;
        }
        arrayDiffDone = true;
      }
      // Check for missing items in array.
      else if (diffToken.value_type == Type::ArrayEnd) {
        while (true) {
          const Token bToken = base.tokens.data[bPos];
          if (bToken.value_type == JS::Type::ArrayEnd)
            break;
          diff.addMissingArrayItems(diffPos, base, bPos);
          base.skip(&bPos);
        }
        arrayDiffDone = true;
      } else {
        diff.set(dPos, DiffType::TypeDiff);
        base.skip(&bPos);
        diff.skip(&dPos);
      }
    }
  }
}

// Note: This function implicitly assumes that the base tokens and the diff tokens represent correct JSON.
inline void diff(const DiffTokens &base, DiffTokens &diff, const DiffOptions &options = {}) {
  if (base.tokens.data.empty() && diff.tokens.data.empty()) {
    // Nothing to do, the diff will be reported as successful.
  } else if (base.tokens.data.empty() && diff.tokens.data.size()) {
    if (diff.tokens.data[0].value_type == Type::ObjectStart)
      diff.setStateForEntireToken(0, DiffType::NewMember);
    else if (diff.tokens.data[0].value_type == Type::ArrayStart)
      diff.setStateForEntireToken(0, DiffType::NewArrayItem);
    else
      setStateForEntireToken(diff, 0, DiffType::ErroneousRootItem);
  } else if (base.tokens.data.size() && diff.tokens.data.empty()) {
    if (diff.tokens.data[0].value_type == Type::ObjectStart)
      diff.addMissingMembers(0, base, 0);
    else if (diff.tokens.data[0].value_type == Type::ArrayStart)
      diff.addMissingArrayItems(0, base, 0);
    else
      setStateForEntireToken(diff, 0, DiffType::ErroneousRootItem);
  } else {
    diff.diff_count = 0;
    const Type &baseType = base.tokens.data[0].value_type;
    const Type &diffType = diff.tokens.data[0].value_type;
    if (baseType == diffType) {
      if (baseType == Type::ObjectStart)
        diffObjects(base, 0, diff, 0, options);
      else if (baseType == Type::ArrayStart)
        diffArrays(base, 0, diff, 0, options);
      else
        setStateForEntireToken(diff, 0, DiffType::ErroneousRootItem);
    } else {
      setStateForEntireToken(diff, 0, DiffType::RootItemDiff);
    }
  }
}
}  // namespace Diff
}  // namespace Internal

struct DiffContext {
  explicit DiffContext(const DiffOptions &options = {})
      : base(), options(options) {}

  explicit DiffContext(const char *baseJson, size_t baseSize, const DiffOptions &options = {})
      : base(baseJson, baseSize), options(options) {
    error = base.error;
  }

  explicit DiffContext(const char *baseJson, const DiffOptions &options = {})
      : base(baseJson, strlen(baseJson)), options(options) {
    error = base.error;
  }

  template <size_t SIZE>
  explicit DiffContext(const char (&baseJson)[SIZE], const DiffOptions &options = {})
      : DiffContext(baseJson, SIZE, options) {}

  explicit DiffContext(const std::string &baseJson, const DiffOptions &options = {})
      : DiffContext(baseJson.c_str(), baseJson.size(), options) {}

  void clear(const DiffOptions &opt = {}) {
    base.clear();
    diffs.clear();
    options = opt;
    error = DiffError::NoError;
  }

  void reset(const char *baseJson, size_t baseSize, const DiffOptions &opt = {}) {
    base.reset(baseJson, baseSize);
    options = opt;
    diffs.clear();
    error = DiffError::NoError;
  }

  void reset(const std::string &baseJson, const DiffOptions &opt = {}) {
    reset(baseJson.c_str(), baseJson.size(), opt);
  }

  void changeBase(const size_t pos) {
    assert(pos < diffs.size());
    DiffTokens tmp = diffs[pos];
    diffs[pos] = base;
    base = tmp;
    invalidate(this->options);
  }

  void invalidate(const DiffOptions &opt) {
    options = opt;
    base.invalidate();
    for (auto &diffTokens : diffs) {
      diffTokens.invalidate();
      diff(diffTokens);
    }
  }

  void invalidate() {
    invalidate(options);
  }

  size_t diff(const char *json, size_t size) {
    DiffTokens diffTokens(json, size);
    error = diffTokens.error;
    if (error != DiffError::NoError)
      return size_t(-1);
    diff(diffTokens);
    diffs.emplace_back(diffTokens);
    return diffs.size() - 1;
  }

  size_t diff(const std::string &json) {
    return diff(json.c_str(), json.size());
  }

  template <size_t SIZE>
  size_t diff(const char (&json)[SIZE]) {
    return diff(json, SIZE);
  }

  void diff(DiffTokens &diffTokens) {
    Internal::Diff::diff(base, diffTokens, options);
  }

  DiffTokens base;
  std::vector<DiffTokens> diffs;
  DiffError error = DiffError::NoError;
  DiffOptions options;
};

}  // namespace JS
#endif  //JSON_STRUCT_DIFF_H
