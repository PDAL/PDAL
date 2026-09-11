#include "DisAndHeightToTarget.hpp"

namespace pdal
{

    static PluginInfo const s_info{
        "filters.disandheighttotarget",
        "cal 2D or 3D distance and height to target pointcloud ,first input pointcloud as target pointcloud ",
        "http://pdal.io/stages/filters.disandheighttotarget.html"};

    CREATE_SHARED_STAGE(DisAndHeightToTarget, s_info)

    std::string DisAndHeightToTarget::getName() const
    {
        return s_info.name;
    }

    void DisAndHeightToTarget::addArgs(ProgramArgs &args)
    {
        args.add("height_dimension", "height to target pointView dimension name ", m_heightToTargetDimName, "HeightToTarget");
        args.add("distance_dimension", "distance  to target pointView dimension name", m_disToTargetDimName, "DistanceToTarget");
        args.add("is3D", "2/3D distance  ", m_is3D, true);
    }

    void DisAndHeightToTarget::addDimensions(PointLayoutPtr layout)
    {
        m_heightToTargetDim = layout->registerOrAssignDim(m_heightToTargetDimName, Dimension::Type::Double);
        m_disToTargetDim = layout->registerOrAssignDim(m_disToTargetDimName, Dimension::Type::Double);
    }

    void DisAndHeightToTarget::prepared(PointTableRef table)
    {
        PointLayoutPtr layout(table.layout());

        if (layout->findDim("X") == Dimension::Id::Unknown ||
            layout->findDim("Y") == Dimension::Id::Unknown ||
            layout->findDim("Z") == Dimension::Id::Unknown)
            throwError("Dimension X Y Z does not all exist.");
    }

    PointViewSet DisAndHeightToTarget::run(PointViewPtr view)
    {
        
         PointViewSet viewSet;
        if (!this->m_target)     
        {
            log()->get(LogLevel::Debug2) << "Adding target points\n";
            this->m_target = view;
        }
      
        else
        {

            KD3Index &kd_fixed3D = this->m_target->build3dIndex();
            KD2Index &kd_fixed2D = this->m_target->build2dIndex();

            for (pdal::PointId i = 0; i < view->size(); ++i)
            {
                PointRef point(*view, i);
                PointIdList indices(1);
                std::vector<double> sqr_dists(1);
                if (m_is3D)
                    kd_fixed3D.knnSearch(point, 1, &indices, &sqr_dists);
                else
                    kd_fixed2D.knnSearch(point, 1, &indices, &sqr_dists);

                double height_to_target =point.getFieldAs<double>(Dimension::Id::Z)- this->m_target->getFieldAs<double>(Dimension::Id::Z, indices[0]) ;
                double distance_to_target = std::sqrt(sqr_dists[0]);
               
                point.setField(m_heightToTargetDim, height_to_target);
                point.setField(m_disToTargetDim, distance_to_target);
            }
            viewSet.insert(view);
            this->m_complete = true;
        }

     
        return viewSet;
    }

    void DisAndHeightToTarget::done(PointTableRef _)
    {
        if (!this->m_complete)
        {
            throw pdal_error(
                "filters.disandheighttotarget  must have two point view inputs, no less");
        }
    }

} // namespace pdal
