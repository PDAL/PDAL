// I3SReader.hpp

#pragma once

#include "../lepcc/src/include/lepcc_c_api.h"
#include "../lepcc/src/include/lepcc_types.h"
#include "i3sReceiver.hpp"

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/PointLayout.hpp>

#include <functional>
#include <queue>
#include <vector>
#include <algorithm>                                                            
#include <chrono>     

#include <json/json.h>
#include <arbiter/arbiter.hpp>
#include <pdal/StageFactory.hpp>


namespace pdal
{
    class FixedPointLayout : public pdal::PointLayout
    {
    public:
        using Added = std::map<std::string, pdal::Dimension::Detail>;
        const Added& added() const { return m_added; }

    private:
        virtual bool update(
                pdal::Dimension::Detail dimDetail,
                const std::string& name) override
        {
            m_added[name] = dimDetail;

            if (!m_finalized)
            {
                if (!contains(m_used, dimDetail.id()))
                {
                    dimDetail.setOffset(m_pointSize);

                    m_pointSize += dimDetail.size();
                    m_used.push_back(dimDetail.id());
                    m_detail[pdal::Utils::toNative(dimDetail.id())] = dimDetail;

                    return true;
                }
            }
            else return m_propIds.count(name);

            return false;
        }

        bool contains(
                const pdal::Dimension::IdList& idList,
                const pdal::Dimension::Id id) const
        {
            for (const auto current : idList)
            {
                if (current == id) return true;
            }

            return false;
        }

        Added m_added;
  };
  class I3SReader : public Reader
  {
  public:
    I3SReader() : Reader() {};
    std::string getName() const;
    void binaryFetch(std::string localUrl);

  private:
    std::unique_ptr<ILeStream> m_stream;
    point_count_t m_index;
    double m_scale_z;
    std::unique_ptr<arbiter::Arbiter> m_arbiter;

    I3SArgs m_args;
    Json::Value m_info;
    FixedPointLayout m_layout;
    std::mutex m_mutex;
    
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table) override;
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);

  };
}
