.. _liblas-to-pdal:

libLAS C API to PDAL transition guide
=====================================

This page shows how to port code using libLAS C API to PDAL API
(which is C++). The new code is not using full power of PDAL but
it uses just what is necessary to read content of a LAS file.


Includes
--------

libLAS include:

.. code-block:: cpp

    #include <liblas/capi/liblas.h>

For PDAL, in addition to PDAL headers, we also include standard headers
which will be useful later:

.. code-block:: cpp

    #include <memory>
    #include <pdal/PointTable.hpp>
    #include <pdal/PointView.hpp>
    #include <pdal/LasReader.hpp>
    #include <pdal/LasHeader.hpp>
    #include <pdal/Options.hpp>

Initial steps
-------------

Opening the dataset in libLAS:

.. code-block:: cpp

    LASReaderH LAS_reader;
    LASHeaderH LAS_header;
    LASSRSH LAS_srs;
    LAS_reader = LASReader_Create(in_opt->answer);
    LAS_header = LASReader_GetHeader(LAS_reader);

The higher level of abstraction in PDAL requires a little bit more code
for the initial steps:

.. code-block:: cpp

    pdal::Option las_opt("filename", in_opt->answer);
    pdal::Options las_opts;
    las_opts.add(las_opt);
    pdal::PointTable table;
    pdal::LasReader las_reader;
    las_reader.setOptions(las_opts);
    las_reader.prepare(table);
    pdal::PointViewSet point_view_set = las_reader.execute(table);
    pdal::PointViewPtr point_view = *point_view_set.begin();
    pdal::Dimension::IdList dims = point_view->dims();
    pdal::LasHeader las_header = las_reader.header();

The PDAL code is also different in the way that we read all the data
right away while in libLAS we just open the file.
To make use of other readers supported by PDAL, see ``StageFactory`` class.

The test if the file was loaded successfully, the test of the header
pointer was used with libLAS:

.. code-block:: cpp

if (LAS_header == NULL) {
    /* fail */
}

In general, PDAL will throw a ``pdal_error`` exception in case something
is wrong and it can't recover such in the case when the file can't be opened.
To handle the exceptional state by yourself, you can wrap the code
in ``try-catch`` block:

.. code-block:: cpp

try {
    /* actual code */
} catch {
    /* fail in your own way */
}


Dataset properties
------------------

We assume we defined all the following variables as ``double``.

The general properties from the LAS file are retrieved from the
header in libLAS:

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

And the same applies PDAL:

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

The point record count in libLAS:

.. code-block:: cpp

    unsigned int n_features = LASHeader_GetPointRecordsCount(LAS_header);

is just point count in PDAL:

.. code-block:: cpp

    unsigned int n_features = las_header.pointCount();

WKT of a spatial reference system is obtained from the header in libLAS:

.. code-block:: cpp

    LAS_srs = LASHeader_GetSRS(LAS_header);
    char* projstr = LASSRS_GetWKT_CompoundOK(LAS_srs);

In PDAL, spatial reference is part of the ``PointTable``:

.. code-block:: cpp

    char* projstr = table.spatialRef().getWKT(pdal::SpatialReference::eCompoundOK).c_str();

Whether the time or color is supported by the LAS format, one would
have to determine from the format ID in libLAS:

.. code-block:: cpp

    las_point_format = LASHeader_GetDataFormatId(LAS_header);
    have_time = (las_point_format == 1 ...

In PDAL, there is a convenient function for it in the header:

.. code-block:: cpp

    have_time = las_header.hasTime();
    have_color = las_header.hasColor();

The presence of color, time and other dimensions can be also determined
with:

.. code-block:: cpp

    pdal::Dimension::IdList dims = point_view->dims();


Iterating over points
---------------------

libLAS:

.. code-block:: cpp

    while ((LAS_point = LASReader_GetNextPoint(LAS_reader)) != NULL) {
        // ...
    }

PDAL:

.. code-block:: cpp

    for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) {
        // ...
    }

Point validity
--------------

The correct usage of libLAS required to test point validity:

.. code-block:: cpp

    LASPoint_IsValid(LAS_point)

In PDAL, there is no need to do that and the caller can assume that
all the points provided by PDAL are valid.


Coordinates
-----------

libLAS:

.. code-block:: cpp

    x = LASPoint_GetX(LAS_point);
    y = LASPoint_GetY(LAS_point);
    z = LASPoint_GetZ(LAS_point);

In PDAL, point coordinates are one of the dimensions:

.. code-block:: cpp

    using namespace pdal::Dimension;
    x = point_view->getFieldAs<double>(Id::X, idx);
    y = point_view->getFieldAs<double>(Id::Y, idx);
    z = point_view->getFieldAs<double>(Id::Z, idx);

Thanks to ``using namespace pdal::Dimension`` we can just write ``Id::X`` etc.


Returns
-------

libLAS:

.. code-block:: cpp

    int return_no = LASPoint_GetReturnNumber(LAS_point);
    int n_returns = LASPoint_GetNumberOfReturns(LAS_point);

PDAL:

.. code-block:: cpp

    int return_no = point_view->getFieldAs<int>(Id::ReturnNumber, idx);
    int n_returns = point_view->getFieldAs<int>(Id::NumberOfReturns, idx);


Classes
-------

libLAS:

.. code-block:: cpp

    int point_class = (int) LASPoint_GetClassification(LAS_point);

PDAL:

.. code-block:: cpp

    int point_class = point_view->getFieldAs<int>(Id::Classification, idx);


Color
-----

libLAS:

.. code-block:: cpp

    LASColorH LAS_color = LASPoint_GetColor(LAS_point);
    int red = LASColor_GetRed(LAS_color);
    int green = LASColor_GetGreen(LAS_color);
    int blue = LASColor_GetBlue(LAS_color);

PDAL:

.. code-block:: cpp

    int red = point_view->getFieldAs<int>(Id::Red, idx);
    int green = point_view->getFieldAs<int>(Id::Green, idx);
    int blue = point_view->getFieldAs<int>(Id::Blue, idx);

For LAS format, ``hasColor()`` method of ``LasHeader`` to see if the
format supports RGB. However, in general, you can test use
``hasDim(Id::Red)``, ``hasDim(Id::Green)`` and ``hasDim(Id::Blue)``
method calls on the point, to see if the color was defined.


Time
----

libLAS:

.. code-block:: cpp

    double time = LASPoint_GetTime(LAS_point);

PDAL:

.. code-block:: cpp

    double time = point_view->getFieldAs<double>(Id::GpsTime, idx);



Other point attributes
----------------------

libLAS:

.. code-block:: cpp

    LASPoint_GetIntensity(LAS_point)
    LASPoint_GetScanDirection(LAS_point)
    LASPoint_GetFlightLineEdge(LAS_point)
    LASPoint_GetScanAngleRank(LAS_point)
    LASPoint_GetPointSourceId(LAS_point)
    LASPoint_GetUserData(LAS_point)

PDAL:

.. code-block:: cpp

    point_view->getFieldAs<int>(Id::Intensity, idx)
    point_view->getFieldAs<int>(Id::ScanDirectionFlag, idx)
    point_view->getFieldAs<int>(Id::EdgeOfFlightLine, idx)
    point_view->getFieldAs<int>(Id::ScanAngleRank, idx)
    point_view->getFieldAs<int>(Id::PointSourceId, idx)
    point_view->getFieldAs<int>(Id::UserData, idx)


Memory management
-----------------

In libLAS C API, we need to explicitly take care of freeing the memory:

.. code-block:: cpp

    LASSRS_Destroy(LAS_srs);
    LASHeader_Destroy(LAS_header);
    LASReader_Destroy(LAS_reader);

When using C++ and PDAL, the objects created on stack free the memory
when they go out of scope. When using smart pointers, they will take
care of the memory they manage. This does not apply to special cases
such as ``exit()`` function calls.
