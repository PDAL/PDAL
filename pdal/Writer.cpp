/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
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

#include <pdal/Writer.hpp>
#include <pdal/Stage.hpp>
#include <pdal/util/Uuid.hpp>

#include <pdal/util/ProgramArgs.hpp>
#include "../filters/private/expr/ConditionalExpression.hpp"

#pragma warning(disable: 4127)  // conditional expression is constant

namespace pdal
{

struct Writer::Args
{
    expr::ConditionalExpression where;
    Arg *whereArg;
    Stage::WhereMergeMode whereMerge;
    Arg *whereMergeArg;
    std::string filename;
};

Writer::Writer() : m_args(new Args)
{}

Writer::~Writer()
{}

const expr::ConditionalExpression* Writer::whereExpr() const
{
    if (!m_args->where.valid())
        return nullptr;
    return &(m_args->where);
}


Stage::WhereMergeMode Writer::mergeMode() const
{
    return m_args->whereMerge;
}

void Writer::l_initialize(PointTableRef table)
{
    Stage::l_initialize(table);
    m_args->filename = replaceTags(m_args->filename);
}

// This is here so that we can make the function final and make sure it
// it isn't used by any subclasses.
void Writer::l_addArgs(ProgramArgs& args)
{
    Stage::l_addArgs(args);

    // Only add the filename argument if this isn't a NoFilenameWriter. (This seemed to me
    // the worst back hack to handle this.)
    if (!dynamic_cast<NoFilenameWriter *>(this))
        args.add("filename", "Output filename", m_args->filename).setPositional();
    m_args->whereArg = &args.add("where", "Expression describing points to be passed to this "
        "filter", m_args->where);
    m_args->whereMergeArg = &args.add("where_merge", "If 'where' option is set, describes "
        "how skipped points should be merged with kept points in standard mode.",
        m_args->whereMerge, WhereMergeMode::Auto);
}

void Writer::l_prepared(PointTableRef table)
{
    Stage::l_prepared(table);
    auto status = m_args->where.prepare(table.layout());
    if (!status)
        throwError("Invalid 'where': " + status.what());
    if (m_args->whereMergeArg->set() && !m_args->whereArg->set())
        throwError("Can't set 'where_merge' options without also setting 'where' option.");
}

std::string Writer::replaceTags(std::string filename)
{
    // Replace '#uuid# placeholder with actual uuid.
    while (auto pos = filename.find("#uuid#") != std::string::npos)
        filename.replace(pos, 6, Uuid().toString());
    return filename;
}

std::string::size_type Writer::handleFilenameTemplate(const std::string& filename)
{
    std::string::size_type suffixPos = filename.find_last_of('.');
    std::string::size_type hashPos = filename.find_first_of('#');
    if (hashPos == std::string::npos)
        return hashPos;

    if (hashPos > suffixPos)
    {
        std::ostringstream oss;
        oss << "Filename template placeholder ('#') is not "
            "allowed in filename suffix.";
        throw pdal_error(oss.str());
    }

    if (filename.find_first_of('#', hashPos + 1) != std::string::npos)
    {
        std::ostringstream oss;
        oss << "Filename specification can only contain "
            "a single '#' template placeholder.";
        throw pdal_error(oss.str());
    }
    return hashPos;
}

std::string Writer::filename() const
{
    return m_args->filename;
}

void Writer::setFilename(const std::string& filename)
{
    m_args->filename = filename;
}

} // namespace pdal
