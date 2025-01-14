
#include "FileSpec.hpp"

#include <nlohmann/json.hpp>

namespace pdal
{

namespace
{

bool extractStringMap(NL::json& node, StringMap& map)
{
    if (!node.is_object())
        return false;
    for (auto& [key, val] : node.items())
    {
        if (val.is_string())
            map.insert({key, val});
        else
            return false;
    }
    return true;
}

} // unnamed namespace

Utils::StatusWithReason FileSpec::parse(NL::json& node)
{
    if (node.is_null())
        return { -1, "'filename' argument contains no data" };
    if (node.is_string())
        m_path = node.get<std::string>();
    else if (node.is_object())
    {
        auto status = extractPath(node);
        if (!status)
            return { -1, status.what() };
        status = extractHeaders(node);
        if (!status)
            return { -1, status.what() };
        status = extractQuery(node);
        if (!status)
            return { -1, status.what() };
        if (!node.empty())
            return { -1, "Invalid item in filename object: " + node.dump() };
    }
    else
        return { -1, "'filename' must be specified as a string." };
    return true;
}

// have to wrap it up here to not expose nlohmann in the header
Utils::StatusWithReason FileSpec::parse(const std::string& jsonOrStr)
{
    // catch json ctor errors?
    NL::json json(jsonOrStr);
    return parse(json);
}

Utils::StatusWithReason FileSpec::extractPath(NL::json& node)
{
    auto it = node.find("path");
    if (it == node.end())
        return { -1, "'filename' object must contain 'path' member." };
    NL::json& val = *it;
    if (!val.is_null())
    {
        if (val.is_string())
            m_path = val.get<std::string>();
        else
            return { -1, "'filename' object 'path' member must be specified as a string." };
        node.erase(it);
    }
    return true;
}

Utils::StatusWithReason FileSpec::extractQuery(NL::json& node)
{
    auto it = node.find("query");
    if (it == node.end())
        return true;
    NL::json& val = *it;
    if (!val.is_null())
    {
        if (!extractStringMap(val, m_query))
            return { -1, "'filename' sub-argument 'query' must be an object of "
                "string key-value pairs." };
    }
    node.erase(it);
    return true;
}

Utils::StatusWithReason FileSpec::extractHeaders(NL::json& node)
{
    auto it = node.find("headers");
    if (it == node.end())
        return true;
    NL::json& val = *it;
    if (!val.is_null())
    {
        auto status = extractStringMap(val, m_headers);
        if (!status)
            return { -1, "'filename' sub-argument 'headers' must be an object of "
                "string key-value pairs." };
    }
    node.erase(it);
    return true;
}

// needs to be converted back to a json string for an Option<T> object to be created.
// This happens after it gets processed by PipelineReaderJSON -- it all feels a bit circular.
std::ostream& operator << (std::ostream& out, const FileSpec& spec)
{
    // some weird stuff was happening in some stages w/o this
    if (spec.onlyFilename())
    {
        out << spec.m_path.string();
        return out;
    }
    
    NL::json json;
    json["path"] = spec.m_path.string();
    if (!spec.m_headers.empty())
        json["headers"] = spec.m_headers;
    if (!spec.m_query.empty())
        json["query"] = spec.m_query;

    out << json;
    return out;
}


} // namespace pdal