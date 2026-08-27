/******************************************************************************
 * Copyright (c) 2022, Kyle Mann (kyle@hobu.co)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#include "Utils.hpp"

#include "../connector/Connector.hpp"

#include <pdal/util/Utils.hpp>

namespace pdal
{

namespace stac
{

pdal_error stac_error(std::string id, std::string stacType,
    std::string const& msg)
{
    return pdal_error("STACError (" + stacType + ": " + id + "): " + msg);
}

pdal_error stac_error(std::string const& msg)
{
    return pdal_error("STACError: " + msg);
}

NL::json loadSchemaJson(const connector::Connector& connector,
    const std::string& url, bool validateSchemaSchema)
{
    // Avoid intermittent failures while resolving remote JSON Schema
    // metaschemas; STAC schemas are still fetched and used for validation.
    if (!validateSchemaSchema &&
        url.find("json-schema.org") != std::string::npos)
        return true;

    return connector.getJson(url);
}

namespace
{

// Length of the "scheme://authority" prefix of `path` or 0 if `path` is not
// a URL. A scheme cannot contain '/', '?' or '#', so a "://" preceded by one
// of those belongs to something else -- a query embedding a URL, for example.
std::string::size_type urlRootLength(const std::string& path)
{
    std::string::size_type schemeEnd = path.find("://");
    if (schemeEnd == std::string::npos)
        return 0;
    if (path.find_first_of("/?#") < schemeEnd)
        return 0;

    // The authority ends at the first '/', '?' or '#' after "scheme://".
    // None at all ("https://host"): the whole string is the root.
    std::string::size_type authEnd = path.find_first_of("/?#", schemeEnd + 3);
    if (authEnd == std::string::npos)
        return path.size();

    return authEnd;
}

// Collapse '.' and '..' segments (RFC 3986 5.2.4). Normalizing is the client's
// job: arbiter hands the path to the store verbatim, and a remote endpoint may
// or may not do it for us the way a filesystem does.
//
// `path` must be rooted -- it starts at '/'. The leading empty segment is what
// stops '..' from walking off the top.
std::string removeDotSegments(const std::string& path)
{
    const StringList segments = Utils::split(path, '/');
    StringList out;
    for (size_t i = 0; i < segments.size(); ++i)
    {
        const std::string& seg = segments[i];
        const bool last = (i + 1 == segments.size());
        if (seg == ".")
        {
            if (last)
                out.push_back("");
        }
        else if (seg == "..")
        {
            // Never pop the leading empty segment -- it marks the root.
            if (out.size() > 1)
                out.pop_back();
            if (last)
                out.push_back("");
        }
        else
            out.push_back(seg);
    }

    std::string resolved;
    for (size_t i = 0; i < out.size(); ++i)
    {
        if (i)
            resolved += '/';
        resolved += out[i];
    }
    return resolved;
}

} // unnamed namespace

namespace StacUtils
{

std::string handleRelativePath(std::string srcPath, std::string linkPath)
{
    // An href carrying its own scheme is absolute -- nothing to resolve.
    if (urlRootLength(linkPath))
        return linkPath;

    const std::string::size_type rootLen = urlRootLength(srcPath);

    // Local source: resolve against the source's directory on the filesystem.
    if (!rootLen)
    {
        //If the filepath is already absolute we don't need to bother.
        if (FileUtils::isAbsolutePath(linkPath))
            return linkPath;

        //If src item isn't an absolute path, we need to convert it to one
        // for getDirectory to work
        if (!FileUtils::isAbsolutePath(srcPath))
            srcPath = FileUtils::toAbsolutePath(srcPath);
        //Get directory of src item
        const std::string baseDir = FileUtils::getDirectory(srcPath);
        //Create absolute path from src item filepath, if it's not already
        // and join relative path to src item's dir path
        return FileUtils::toAbsolutePath(linkPath, baseDir);
    }

    // Remote source: resolve the reference against the source URL rather than
    // against the local filesystem, which is what toAbsolutePath() would do.
    const bool rooted = Utils::startsWith(linkPath, "/");

    // Dot-segment removal applies only to the path, so detach any query or
    // fragment the reference carries and re-attach it once normalized. This
    // must happen before the absolute-path check below: its "://" heuristic
    // would otherwise misread a URL embedded in the query as a scheme.
    std::string refSuffix;
    const std::string::size_type refEnd = linkPath.find_first_of("?#");
    if (refEnd != std::string::npos)
    {
        refSuffix = linkPath.substr(refEnd);
        linkPath = linkPath.substr(0, refEnd);
    }

    // A path the filesystem layer already calls absolute without it being
    // rooted -- a Windows drive letter -- isn't a reference we can resolve
    // against a URL. Pass it through, as we did before.
    if (!rooted && FileUtils::isAbsolutePath(linkPath))
        return linkPath + refSuffix;

    const std::string root = srcPath.substr(0, rootLen);  // scheme://authority

    // A reference starting at the root replaces the base path entirely.
    if (rooted)
        return root + removeDotSegments(linkPath) + refSuffix;

    std::string base = srcPath.substr(rootLen);  // /path[?query][#fragment]

    // A reference that is only a query or fragment names the base resource
    // itself, not its directory (RFC 3986 5.3): keep the base's whole path,
    // and for a fragment-only reference keep the base's query too.
    if (linkPath.empty() && !refSuffix.empty())
    {
        if (refSuffix[0] == '#')
            return root + base.substr(0, base.find('#')) + refSuffix;
        return root + base.substr(0, base.find_first_of("?#")) + refSuffix;
    }

    // A relative reference inherits neither the base's query nor its fragment.
    base = base.substr(0, base.find_first_of("?#"));

    // Keep everything up to and including the base's last '/' -- the
    // directory the relative reference hangs off of. No '/' at all means
    // the source URL has no path ("https://host"): resolve from the root.
    const std::string::size_type lastSlash = base.find_last_of('/');
    if (lastSlash == std::string::npos)
        base = "/";
    else
        base = base.substr(0, lastSlash + 1);

    return root + removeDotSegments(base + linkPath) + refSuffix;
}

std::time_t getStacTime(std::string in)
{
    std::istringstream dateStr(in);
    std::tm date {};
    dateStr >> std::get_time(&date, "%Y-%m-%dT%H:%M:%S");
    if (dateStr.fail())
        throw stac_error("Specified date (" + dateStr.str() +
            ") cannot be parsed. Dates must fit RFC 3339 specs.");
    return std::mktime(&date);
}

std::string stacId(const NL::json& stac)
{
    std::stringstream msg;
    try
    {
        return stac.at("id").get<std::string>();
    }
    catch (NL::detail::out_of_range& e)
    {
        msg << "Missing required key 'id'. " << e.what();
        throw pdal_error(msg.str());
    }
    catch (NL::detail::type_error& e)
    {
        msg << "Required key 'id' is not of type 'string'. " << e.what();
        throw pdal_error(msg.str());
    }
}

std::string stacType(const NL::json& stac)
{
    try
    {
        return stac.at("type").get<std::string>();
    }
    catch (NL::detail::out_of_range& e)
    {
        std::stringstream msg;
        msg << "Missing required key 'type'. " << e.what();
        throw pdal_error(msg.str());
    }
    catch (NL::detail::type_error& e)
    {
        std::stringstream msg;
        msg << "Invalid key value 'type'. " << e.what();
        throw pdal_error(msg.str());
    }
}

std::string icSelfPath(const NL::json& json)
{
    try
    {
        NL::json links = Utils::jsonValue(json, "links");
        for (const NL::json& link: links)
        {
            std::string target = Utils::jsonValue<std::string>(link, "rel");
            if (target == "self")
                return Utils::jsonValue<std::string>(link, "href");
        }
    }
    catch(std::runtime_error& )
    {
        return "";
    }

    return "";
}


}//StacUtils
}//stac
}//pdal
