#pragma once

#include <pdal/Filter.hpp>
#include <pdal/KDIndex.hpp>

namespace pdal
{

    class PDAL_DLL DisAndHeightToTarget : public Filter
    {
    public:
        DisAndHeightToTarget() : Filter() ,m_target(nullptr) ,m_complete(false)
        {}
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs &args);
        virtual void addDimensions(PointLayoutPtr layout);
        virtual void prepared(PointTableRef table);
        virtual PointViewSet run(PointViewPtr view);
        virtual void done(PointTableRef _);

        DisAndHeightToTarget &operator=(const DisAndHeightToTarget &); // not implemented
        DisAndHeightToTarget(const DisAndHeightToTarget &);            // not implemented

    private:

        std::string m_heightToTargetDimName; //height to target pointView dimension name
        std::string m_disToTargetDimName; //distance  to target pointView dimension name
        bool m_is3D;          //2D or 3D defalut true

        Dimension::Id m_heightToTargetDim;   //height to target pointView dimension ID
        Dimension::Id m_disToTargetDim;   //distance  to target pointView dimension ID

        bool m_complete;        //judge input pointview size >=2
        PointViewPtr m_target; //reference target pointView

      
       



    };

} // namespace pdal
