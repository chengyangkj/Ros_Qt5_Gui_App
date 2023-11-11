#include <string>
#include <json_struct/json_struct_diff.h>

const char jsonBase[] = R"json(
{
    "key" : "value",
    "number" : 100,
    "value" : 1.23456,
    "boolean" : true
}
)json";

const char jsonDiff[] = R"json(
{
    "key" : "val",
    "number" : 100,
    "value" : 1.23457,
    "boolean" : 0
}
)json";

int main()
{
    fprintf(stdout, "Performing JSON diff between\n%s\nand\n%s\n...\n\n", jsonBase, jsonDiff);

    JS::DiffOptions diffOptions { JS::DiffFlags::FuzzyFloatComparison, 1e-6 }; // 1e-5 would give no floating point diff in this example.
    JS::DiffContext diffContext(jsonBase, diffOptions);
    diffContext.diff(jsonDiff);

    if (diffContext.error != JS::DiffError::NoError)
    {
        fprintf(stderr, "Error while diffing JSON: %d\n", static_cast<int>(diffContext.error));
        return 1;
    }

    const JS::DiffTokens& baseTokens = diffContext.base;
    const JS::DiffTokens& diffTokens = diffContext.diffs[0]; // We only diffed one JSON string, multiple diffs are supported if needed.
    fprintf(stdout, "Number of diffs: %zu\n", diffTokens.diff_count);

    if (baseTokens.size() != diffTokens.size())
    {
        fprintf(stderr, "The size of the baseTokens and diffTokens must be the same in this example.");
        return 2;
    }

    // This loop assumes that baseTokens.size() == diffTokens.size() as is the case in this example.
    for (size_t i = 0; i < diffTokens.size(); i++)
    {
        const JS::Token baseToken = baseTokens.tokens.data[i];
        const JS::Token diffToken = diffTokens.tokens.data[i];
        JS::DiffType diffType = diffTokens.diffs[i];

        std::string baseValue(baseToken.value.data, baseToken.value.size);
        std::string diffValue(diffToken.value.data, diffToken.value.size);

        switch (diffType)
        {
            case JS::DiffType::NoDiff:
            {
                fprintf(stdout, "No diff: %s\n", baseValue.c_str());
                break;
            }
            case JS::DiffType::ValueDiff:
            {
                fprintf(stdout, "Value diff: base = %s, diff = %s\n", baseValue.c_str(), diffValue.c_str());
                break;
            }
            case JS::DiffType::TypeDiff:
            {
                fprintf(stdout, "Type diff: base = %s, diff = %s\n", baseValue.c_str(), diffValue.c_str());
                break;
            }
            case JS::DiffType::NewMember:
            case JS::DiffType::NewArrayItem:
            case JS::DiffType::MissingMembers:
            case JS::DiffType::MissingArrayItems:
            case JS::DiffType::RootItemDiff:
            case JS::DiffType::ErroneousRootItem:
            default:
            {
                fprintf(stdout, "Other diff type :%d.\n", static_cast<int>(diffType)); // Does not occur in this example.
                break;
            }
        }
    }

    return 0;
}
