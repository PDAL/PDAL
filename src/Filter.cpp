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

#include <pdal/Filter.hpp>

namespace pdal
{


Filter::Filter(Stage& prevStage, const Options& options)
    : Stage(options)
    , m_prevStage(prevStage)
{
    return;
}


void Filter::initialize()
{
    getPrevStage().initialize();

    // by default, we set our core properties to be the same as those 
    // of the previous stage
    this->setCoreProperties(getPrevStage());

    Stage::initialize();

    return;
}


const Stage& Filter::getPrevStage() const
{
    return m_prevStage;
}


Stage& Filter::getPrevStage()
{
    return m_prevStage;
}


boost::property_tree::ptree Filter::generatePTree() const
{
    boost::property_tree::ptree tree;

    tree.add("Type", getName());
    
    boost::property_tree::ptree optiontree = getOptions().getPTree();
    tree.add_child(optiontree.begin()->first, optiontree.begin()->second);

    const Stage& stage = getPrevStage();
    boost::property_tree::ptree subtree = stage.generatePTree();

    tree.add_child(subtree.begin()->first, subtree.begin()->second);
    
    boost::property_tree::ptree root;
    root.add_child("Filter", tree);

    return root;
}


boost::property_tree::ptree Filter::toPTree() const
{
    boost::property_tree::ptree tree = Stage::toPTree();

    // (nothing to add for a Filter)

    return tree;
}


} // namespace pdal
