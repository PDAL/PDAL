
#include "FileSpec.hpp"

#include <nlohmann/json.hpp>

namespace pdal
{

Utils::StatusWithReason FileSpec::parse(NL::json& json)
{
    try
    {
        extractPath(json);
        extractHeaders(json);
        extractQuery(json);
    }
    catch(const std::exception& e)
    {
        return Utils::StatusWithReason(-1, e.what());
    }
    return true;
}

void FileSpec::extractPath(NL::json& node)
{
    auto it = node.find("path");
    if (it == node.end())
          throw pdal_error("JSON pipeline: 'filename' object must contain 'path' member.");

    NL::json& val = *it;
    if (!val.is_null())
    {
        if (val.is_string())
            m_path = val.get<std::string>();
        else
            throw pdal_error("JSON pipeline: filename 'path' member must be specified "
                "as a string.");
        node.erase(it);
    }
}

void FileSpec::extractQuery(NL::json& node)
{
    auto it = node.find("headers");
    if (it == node.end())
        return;

    NL::json& val = *it;
    if (!val.is_null())
    {
        m_headers = extractStringMap("headers", val);
        node.erase(it);
    }
}

void FileSpec::extractHeaders(NL::json& node)
{
    auto it = node.find("query");
    if (it == node.end())
        return;

    NL::json& val = *it;
    if (!val.is_null())
    {
        m_query = extractStringMap("query", val);
        node.erase(it);
    }
}

StringMap FileSpec::extractStringMap(const std::string& name, NL::json& node)
{
    StringMap smap;

    auto error = [&name]()
    {
        throw pdal_error("JSON pipeline: '" + name + "' must be an object of "
            "string key-value pairs.");
    };

    if (node.is_object())
    {
        for (auto& [key, val] : node.items())
        {
            if (val.is_string())
                smap.insert({key, val});
            else
                error();
        }
    }
    else
        error();
    return smap;
}

} // namespace pdal