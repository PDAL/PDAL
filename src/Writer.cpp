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
#include <pdal/UserCallback.hpp>

#include <pdal/PipelineWriter.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

namespace pdal
{

void Writer::writerProcessOptions(const Options& options)
{
    auto setOffset = [options](XForm& xform, const std::string& opName)
    {
        if (!options.hasOption(opName))
            return;
        std::string offset = options.getValueOrThrow<std::string>(opName);
        if (offset == "auto")
            xform.m_autoOffset = true;
        else
            xform.m_offset = boost::lexical_cast<double>(offset);
    };

    setOffset(m_xXform, "offset_x");
    setOffset(m_yXform, "offset_y");
    setOffset(m_zXform, "offset_z");

    if (options.hasOption("filename"))
        m_filename = options.getValueOrThrow<std::string>("filename");
    if (options.hasOption("scale_x"))
        m_xXform.m_scale = options.getValueOrThrow<double>("scale_x");
    if (options.hasOption("scale_y"))
        m_yXform.m_scale = options.getValueOrThrow<double>("scale_y");
    if (options.hasOption("scale_z"))
        m_zXform.m_scale = options.getValueOrThrow<double>("scale_z");
    m_outputDims = options.getValueOrDefault<StringList>("output_dims");
}


void Writer::setAutoOffset(const PointViewPtr view)
{
   if (!m_xXform.m_autoOffset && !m_yXform.m_autoOffset &&
       !m_zXform.m_autoOffset)
       return;
   if (view->empty())
        return;

    if (m_xXform.m_autoOffset)
        m_xXform.m_offset = (std::numeric_limits<double>::max)();
    if (m_yXform.m_autoOffset)
        m_yXform.m_offset = (std::numeric_limits<double>::max)();
    if (m_zXform.m_autoOffset)
        m_zXform.m_offset = (std::numeric_limits<double>::max)();
    for (PointId idx = 0; idx < view->size(); idx++)
    {
        if (m_xXform.m_autoOffset)
            m_xXform.m_offset =
                std::min(view->getFieldAs<double>(Dimension::Id::X, idx),
                    m_xXform.m_offset);
        if (m_yXform.m_autoOffset)
            m_yXform.m_offset =
                std::min(view->getFieldAs<double>(Dimension::Id::Y, idx),
                    m_yXform.m_offset);
        if (m_zXform.m_autoOffset)
            m_zXform.m_offset =
                std::min(view->getFieldAs<double>(Dimension::Id::Z, idx),
                    m_zXform.m_offset);
    }
}


boost::property_tree::ptree Writer::serializePipeline() const
{
    boost::property_tree::ptree tree;

    tree.add("<xmlattr>.type", getName());

    PipelineWriter::write_option_ptree(tree, getOptions());

    const Stage& stage = *getInputs()[0];
    boost::property_tree::ptree subtree = stage.serializePipeline();

    tree.add_child(subtree.begin()->first, subtree.begin()->second);

    boost::property_tree::ptree root;
    root.add_child("Writer", tree);

    return root;
}

} // namespace pdal
