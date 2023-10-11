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

#include <time.h>
#include <ctime>

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/JsonFwd.hpp>
#include <pdal/StageFactory.hpp>
#include <filters/MergeFilter.hpp>


namespace pdal
{

namespace stac
{
    class Item;
    class Catalog;
}

class PDAL_DLL StacReader : public Reader, public Streamable
{
    public:

        StacReader();
        ~StacReader();

        std::string getName() const override;

        using StringMap = std::map<std::string, std::string>;

    private:

        struct Private;
        struct Args;

        std::unique_ptr<Args> m_args;
        std::unique_ptr<Private> m_p;

        StageFactory m_factory;

        void printErrors(stac::Catalog& c);
        void handleNested(stac::Catalog& c);
        void addItem(stac::Item& item);
        void handleItem(NL::json stacJson, std::string itemPath);
        void handleCatalog(NL::json stacJson, std::string catPath);
        void handleCollection(NL::json stacJson, std::string colPath);
        void handleItemCollection(NL::json stacJson, std::string icPath);
        void initializeArgs();
        void setConnectionForwards(StringMap& headers, StringMap& query);

        virtual void initialize() override;
        virtual void addArgs(ProgramArgs& args) override;
        virtual QuickInfo inspect() override;
        virtual void prepared(PointTableRef table) override;
        virtual void ready(PointTableRef table) override;
        virtual point_count_t read(PointViewPtr view, point_count_t num) override;
        virtual bool processOne(PointRef& point) override;
        virtual PointViewSet run(PointViewPtr view) override;

        MergeFilter m_merge;

};
}
