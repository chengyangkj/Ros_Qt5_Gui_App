#include <string>
#include <vector>
#include <json_struct/json_struct_diff.h>

const char jsonBase[] = R"json(
[
    {
        "key" : "val1",
        "number" : 1,
        "value" : 1.23456,
        "boolean" : true
    },
    {
        "key" : "val2",
        "boolean" : false
    },
    {
        "key" : "val3",
        "number" : 3,
        "value" : 1.23456,
        "boolean" : false
    },
    123
]
)json";

const char jsonDiff[] = R"json(
[
    {
        "number" : 0,
        "key" : "val1",
        "value" : 1.23456001
    },
    {
        "key" : "val2",
        "number" : 2,
        "boolean" : false,
        "value" : 1.23456
    },
    {
        "_number" : 3,
        "key" : "val3",
        "_value" : 1.23456,
        "_boolean" : false
    },
    1.23,
    "NewArrayItem"
]
)json";

bool isKeyValuePair(const JS::Token& token)
{
    return token.name.size != 0;
}

int main()
{
    fprintf(stdout, "Performing JSON diff between\n%s\nand\n%s\n...\n\n", jsonBase, jsonDiff);

    JS::DiffContext diffContext(jsonBase);
    size_t diffPos = diffContext.diff(jsonDiff);

    if (diffContext.error != JS::DiffError::NoError)
    {
        fprintf(stderr, "Error while diffing JSON: %d\n", static_cast<int>(diffContext.error));
        return 1;
    }

    const JS::DiffTokens& diffTokens = diffContext.diffs[diffPos];
    fprintf(stdout, "Number of diffs: %zu\n", diffTokens.diff_count);
    
    for (size_t i = 0; i < diffTokens.size(); i++)
    {
        const JS::Token& diffToken = diffTokens.tokens.data[i];
        JS::DiffType diffType = diffTokens.diffs[i];

        // Both of these may be empty.
        std::string key(diffToken.name.data, diffToken.name.size);
        std::string value(diffToken.value.data, diffToken.value.size);

        switch (diffType)
        {
            case JS::DiffType::NoDiff:
            {
                if (isKeyValuePair(diffToken))
                    fprintf(stdout, "No diff: %s: %s\n", key.c_str(), value.c_str());
                else
                    fprintf(stdout, "No diff: %s\n", value.c_str());
                break;
            }
            case JS::DiffType::ValueDiff:
            {
                if (isKeyValuePair(diffToken))
                    fprintf(stdout, "Value diff: %s: %s\n", key.c_str(), value.c_str());
                else
                    fprintf(stdout, "Value diff: %s\n", value.c_str());
                break;
            }
            case JS::DiffType::TypeDiff:
            {
                fprintf(stdout, "Type diff: %s\n", value.c_str());
                break;
            }
            case JS::DiffType::NewMember:
            {
                fprintf(stdout, "New member: %s: %s\n", key.c_str(), value.c_str());
                break;
            }
            case JS::DiffType::NewArrayItem:
            {
                // Note: No new objects or arrays in this example. Otherwise we must check for
                // diffType == JS::Type::ObjectStart or diffType == JS::Type::ArrayStart and iterate
                // through that object or array.
                fprintf(stdout, "New array item: %s: %s\n", key.c_str(), value.c_str());
                break;
            }
            case JS::DiffType::MissingMembers:
            {
                const auto* missingMembers = diffTokens.getMissingMembers(diffToken);
                for (const auto& m : *missingMembers)
                {
                    std::string missingStr = "\"" + std::string(m.name.data, m.name.size) + "\"";
                    fprintf(stdout, "Missing member: %s\n", missingStr.c_str());
                }
                break;
            }
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
