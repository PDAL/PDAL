.. _development_goals:

================================================================================
Project Goals
================================================================================

1. PDAL is a library which provides APIs for reading, writing, and
   processing geospatial point cloud data of various formats.  Additionally, some
   command line tools are provided.  As `GDAL`_ is to 2D pixels, `OGR`_ is to
   geospatial vector geometry, PDAL is to multidimensional points.

2. From a market perspective, PDAL is "version 2" of `libLAS`_.  The actual
   code base will be different, however, and the APIs will not be
   compatible.

3. The PDAL implementation has high performance, yet the API remains
   flexible.  We recognize that these two goals will conflict at times and
   will weigh the tradeoffs pragmatically.

4. The architecture of a PDAL-based workflow will be a pipeline of
   connected stages, each stage being either a data source (such as a file
   reader), a filter (such as a point thinner), or data sink (such as a
   file writer).

5. The PDAL library will be written in C++. PDAL will support multiple platforms, specifically
   Windows, Linux, and Mac.

6. PDAL is open source and is released under a BSD license. See :ref:`license` for
   more information.

.. _`GDAL`: http://gdal.org
.. _`OGR`: http://gdal.org/ogr/
.. _`libLAS`: https://www.liblas.org
