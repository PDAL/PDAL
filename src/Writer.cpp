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
        if (options.hasOption(opName))
            xform.setOffset(options.getValueOrThrow<std::string>(opName));
    };

    auto setScale = [options](XForm& xform, const std::string& opName)
    {
        if (options.hasOption(opName))
            xform.setScale(options.getValueOrThrow<std::string>(opName));
    };

    setOffset(m_xXform, "offset_x");
    setOffset(m_yXform, "offset_y");
    setOffset(m_zXform, "offset_z");

    setScale(m_xXform, "scale_x");
    setScale(m_yXform, "scale_y");
    setScale(m_zXform, "scale_z");

    if (options.hasOption("filename"))
        m_filename = options.getValueOrThrow<std::string>("filename");
    m_outputDims = options.getValueOrDefault<StringList>("output_dims");
}


void Writer::setAutoXForm(const PointViewPtr view)
{
   double xmin = (std::numeric_limits<double>::max)();
   double xmax = (std::numeric_limits<double>::lowest)();
   bool xmod = m_xXform.m_autoOffset || m_xXform.m_autoScale;

   double ymin = (std::numeric_limits<double>::max)();
   double ymax = (std::numeric_limits<double>::lowest)();
   bool ymod = m_yXform.m_autoOffset || m_yXform.m_autoScale;

   double zmin = (std::numeric_limits<double>::max)();
   double zmax = (std::numeric_limits<double>::lowest)();
   bool zmod = m_zXform.m_autoOffset || m_zXform.m_autoScale;

   if (!xmod && !ymod && !zmod)
       return;
   if (view->empty())
        return;

    for (PointId idx = 0; idx < view->size(); idx++)
    {
        if (xmod)
        {
            double x = view->getFieldAs<double>(Dimension::Id::X, idx);
            xmin = std::min(x, xmin);
            xmax = std::max(x, xmax);
        }
        if (ymod)
        {
            double y = view->getFieldAs<double>(Dimension::Id::Y, idx);
            ymin = std::min(y, ymin);
            ymax = std::max(y, ymax);
        }
        if (zmod)
        {
            double z = view->getFieldAs<double>(Dimension::Id::Z, idx);
            zmin = std::min(z, zmin);
            zmax = std::max(z, zmax);
        }
    }

    if (m_xXform.m_autoOffset)
    {
        m_xXform.m_offset = xmin;
        xmax -= xmin;
    }
    if (m_yXform.m_autoOffset)
    {
        m_yXform.m_offset = ymin;
        ymax -= ymin;
    }
    if (m_zXform.m_autoOffset)
    {
        m_zXform.m_offset = zmin;
        zmax -= zmin;
    }
    if (m_xXform.m_autoScale)
        m_xXform.m_scale = xmax / (std::numeric_limits<int>::max)();
    if (m_yXform.m_autoScale)
        m_yXform.m_scale = ymax / (std::numeric_limits<int>::max)();
    if (m_zXform.m_autoScale)
        m_zXform.m_scale = zmax / (std::numeric_limits<int>::max)();
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
