.. _writing-writer:

libLAS C API to PDAL
====================



Includes
--------

.. code-block:: cpp

    #include <liblas/capi/liblas.h>

.. code-block:: cpp

    #include <memory>
    #include <pdal/PointTable.hpp>
    #include <pdal/PointView.hpp>
    #include <pdal/StageFactory.hpp>
    #include <pdal/LasReader.hpp>
    #include <pdal/LasHeader.hpp>
    #include <pdal/Options.hpp>

Opening the dataset
-------------------

.. code-block:: cpp

    LASReaderH LAS_reader;
    LASHeaderH LAS_header;
    LASSRSH LAS_srs;
    LASPointH LAS_point;
    double scale_x, scale_y, scale_z, offset_x, offset_y, offset_z;
    int las_point_format;
    LAS_reader = LASReader_Create(in_opt->answer);
    LAS_header = LASReader_GetHeader(LAS_reader);

.. code-block:: cpp

    pdal::Option las_opt("filename", in_opt->answer);
    pdal::Options las_opts;
    las_opts.add(las_opt);
    pdal::PointTable table;
    std::unique_ptr<pdal::LasReader> las_reader(new pdal::LasReader())
    las_reader->setOptions(las_opts);
    las_reader->prepare(table);
    pdal::PointViewSet point_view_set = las_reader->execute(table);
    pdal::PointViewPtr point_view = *point_view_set.begin();
    pdal::Dimension::IdList dims = point_view->dims();

here we read right away and then iterate over it

TODO: How to test if loaded successfully? Was ``LAS_header == NULL``

TODO: When the following can be used?

.. code-block:: cpp

    pdal::StageFactory f;
    std::unique_ptr<pdal::Stage> las_reader(f.createStage("readers.las")

Dataset properties
------------------

.. code-block:: cpp

    scale_x = LASHeader_GetScaleX(LAS_header);
    scale_y = LASHeader_GetScaleY(LAS_header);
    scale_z = LASHeader_GetScaleZ(LAS_header);

    offset_x = LASHeader_GetOffsetX(LAS_header);
    offset_y = LASHeader_GetOffsetY(LAS_header);
    offset_z = LASHeader_GetOffsetZ(LAS_header);

    xmin = LASHeader_GetMinX(LAS_header);
    xmax = LASHeader_GetMaxX(LAS_header);
    ymin = LASHeader_GetMinY(LAS_header);
    ymax = LASHeader_GetMaxY(LAS_header);


.. code-block:: cpp


    scale_x = las_header.scaleX();
    scale_y = las_header.scaleY();
    scale_z = las_header.scaleZ();

    offset_x = las_header.offsetX();
    offset_y = las_header.offsetY();
    offset_z = las_header.offsetZ();

    xmin = las_header.minX();
    xmax = las_header.maxX();
    ymin = las_header.minY();
    ymax = las_header.maxY();


.. code-block:: cpp

    n_features = LASHeader_GetPointRecordsCount(LAS_header);

.. code-block:: cpp

    n_features = las_header.pointCount();

.. code-block:: cpp

    LAS_srs = LASHeader_GetSRS(LAS_header);
    char* projstr = LASSRS_GetWKT_CompoundOK(LAS_srs);

.. code-block:: cpp

    char* projstr = table.spatialRef().getWKT(pdal::SpatialReference::eCompoundOK).c_str();

.. code-block:: cpp

    las_point_format = LASHeader_GetDataFormatId(LAS_header);
    have_time = (las_point_format == 1 ...

.. code-block:: cpp

    have_time = las_header.hasTime();
    have_color = las_header.hasColor();


Iterating over points
---------------------

.. code-block:: cpp

    while ((LAS_point = LASReader_GetNextPoint(LAS_reader)) != NULL) {
        // ...
    }

.. code-block:: cpp

    for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) {
        // ...
    }

Point validity
--------------

.. code-block:: cpp

    LASPoint_IsValid(LAS_point)

TODO: do we need to do that?


Coordinates
-----------

.. code-block:: cpp

    x = LASPoint_GetX(LAS_point);
    y = LASPoint_GetY(LAS_point);
    z = LASPoint_GetZ(LAS_point);

.. code-block:: cpp

    using namespace pdal::Dimension;
    x = point_view->getFieldAs<double>(Id::X, idx);
    y = point_view->getFieldAs<double>(Id::Y, idx);
    z = point_view->getFieldAs<double>(Id::Z, idx);

Returns
-------

.. code-block:: cpp

    int return_no = LASPoint_GetReturnNumber(LAS_point);
    int n_returns = LASPoint_GetNumberOfReturns(LAS_point);

.. code-block:: cpp

    int return_no = point_view->getFieldAs<int>(Id::ReturnNumber, idx);
    int n_returns = point_view->getFieldAs<int>(Id::NumberOfReturns, idx);


Classes
-------

.. code-block:: cpp

    int point_class = (int) LASPoint_GetClassification(LAS_point);

.. code-block:: cpp

    int point_class = point_view->getFieldAs<int>(Id::Classification, idx);


Color
-----

.. code-block:: cpp

    LASColorH LAS_color = LASPoint_GetColor(LAS_point);
    int red = LASColor_GetRed(LAS_color);
    int green = LASColor_GetGreen(LAS_color);
    int blue = LASColor_GetBlue(LAS_color);


.. code-block:: cpp

    int red = point_view->getFieldAs<int>(Id::Red, idx);
    int green = point_view->getFieldAs<int>(Id::Green, idx);
    int blue = point_view->getFieldAs<int>(Id::Blue, idx);


TODO: what is relation of ``hasColor()`` and ``hasDim(Id::Red)`` ?


Time
----

.. code-block:: cpp

    double time = LASPoint_GetTime(LAS_point);

.. code-block:: cpp

    double time = point_view->getFieldAs<double>(Id::GpsTime, idx);



Other point attributes
----------------------

.. code-block:: cpp

    LASPoint_GetIntensity(LAS_point)
    LASPoint_GetScanDirection(LAS_point)
    LASPoint_GetFlightLineEdge(LAS_point)
    LASPoint_GetScanAngleRank(LAS_point)
    LASPoint_GetPointSourceId(LAS_point)
    LASPoint_GetUserData(LAS_point)

.. code-block:: cpp

    point_view->getFieldAs<int>(Id::Intensity, idx)
    point_view->getFieldAs<int>(Id::ScanDirectionFlag, idx)
    point_view->getFieldAs<int>(Id::EdgeOfFlightLine, idx)
    point_view->getFieldAs<int>(Id::ScanAngleRank, idx)
    point_view->getFieldAs<int>(Id::PointSourceId, idx)
    point_view->getFieldAs<int>(Id::UserData, idx)


Memory management
-----------------

.. code-block:: cpp

    LASSRS_Destroy(LAS_srs);
    LASHeader_Destroy(LAS_header);
    LASReader_Destroy(LAS_reader);


should go out of scope

TODO: we need something for exit [noreturn]

we don't destroy for PDAL as we will just go out of scope
(using stack or smart pointers)
