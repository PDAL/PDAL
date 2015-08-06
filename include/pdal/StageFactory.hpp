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

#pragma once

#include <pdal/pdal_export.hpp>
#include <pdal/Stage.hpp>

#include <map>
#include <string>
#include <vector>


namespace pdal
{

class Options;


// This class provides a mechanism for creating Stage objects using only a
// string (the stage type name) and an Options block.
//
// We keep a list of (stage type name, creation function) pairs, which
// acts as a registry of creator functions.  The list is initialized with
// the core stages we know about (I wish C++ had anonymous functions.).
// We allow the user to add his own "external" drivers to the registry list
// as well.
//
// We use 4 different functions for each kind of operation, since we have
// 4 types of derived classes from Stage and they all have slightly different
// parameters.  That makes it kinda messy.

class PDAL_DLL StageFactory
{
public:
    StageFactory(bool no_plugins = true);

    // infer the driver to use based on filename extension
    // returns "" if no driver found
    static std::string inferReaderDriver(const std::string& filename);

    // infer the driver to use based on filename extension
    // returns "" if no driver found
    static std::string inferWriterDriver(const std::string& filename);

    // modify options based upon expectations implicit in a given filename
    // e.g. output files ending in .laz should be compressed
    static pdal::Options inferWriterOptionsChanges(const std::string& filename);

    Stage *createStage(const std::string& type, bool ownStage = false);

    StringList getStageNames() const;
    std::map<std::string, std::string> getStageMap() const;

private:
    StageFactory& operator=(const StageFactory&); // not implemented
    StageFactory(const StageFactory&); // not implemented

    std::vector<std::unique_ptr<Stage>> m_ownedStages;
};

} // namespace pdal

