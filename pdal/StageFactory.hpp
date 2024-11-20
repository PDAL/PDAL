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
#include <mutex>
#include <string>
#include <vector>

namespace pdal
{

class Options;

/**
  This class provides a mechanism for creating Stage objects given a driver
  name.  Creates stages are owned by the factory and destroyed when the
  factory is destroyed.  Stages can be explicitly destroyed with destroyStage()
  if desired.

  \note  Stage creation is thread-safe.
*/
class PDAL_EXPORT StageFactory
{
public:
    /**
      Create a stage factory.

      \param ignored  Ignored argument.
    */
    StageFactory(bool ignored = true);

    /**
      Infer the reader to use based on a filename.

      \param filename  Filename that should be analyzed to determine a driver.
      \return  Driver name or empty string if no reader can be inferred from
        the filename.
    */
    static std::string inferReaderDriver(const std::string& filename);

    /**
      Infer the writer to use based on filename extension.

      \return  Driver name or empty string if no writer can be inferred from
        the filename.
    */
    static std::string inferWriterDriver(const std::string& filename);

    /**
      Create a stage and return a pointer to the created stage.
      The factory takes ownership of any successfully created stage.

      \param stage_name  Type of stage to by created.
      \return  Pointer to created stage.
    */
    Stage *createStage(const std::string& type);

    /**
      Destroy a stage created by this factory.  This doesn't need to be
      called unless you specifically want to destroy a stage as all stages
      are destroyed when the factory is destroyed.

      \param stage  Pointer to stage to destroy.
    */
    void destroyStage(Stage *stage);

private:
    StageFactory& operator=(const StageFactory&); // not implemented
    StageFactory(const StageFactory&); // not implemented

    std::vector<std::unique_ptr<Stage>> m_ownedStages;
    std::mutex m_mutex;
};

} // namespace pdal

