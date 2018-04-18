.. _writers:

Writers
=======

Writers consume data provided by :ref:`readers`. Some writers can consume any
dimension type, while others only understand fixed dimension names.

.. note::

    PDAL predefined dimension names can be found in the dimension registry:
    :ref:`dimensions`

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   writers.bpf
   writers.gdal
   writers.geowave
   writers.greyhound
   writers.las
   writers.matlab
   writers.nitf
   writers.null
   writers.oci
   writers.ogr
   writers.pcd
   writers.pgpointcloud
   writers.ply
   writers.sbet
   writers.sqlite
   writers.text

:ref:`writers.bpf`
    Write BPF version 3 files. BPF is an NGA specification for point cloud data.

:ref:`writers.gdal`
    Create a raster from a point cloud using an interpolation algorithm.

:ref:`writers.geowave`
    Write point cloud data to Accumulo.

:ref:`writers.greyhound`
    Append new dimensions (or update existing dimensions) onto a Greyhound
    resource. Must be used along with a Greyhound reader, and intermediate
    filters that cull points are not allowed.

:ref:`writers.las`
    Write ASPRS LAS versions 1.0 - 1.4 formatted data. LAZ support is also
    available if enabled at compile-time.

:ref:`writers.matlab`
    Write MATLAB .mat files. The output has a single array struct.

:ref:`writers.nitf`
    Write LAS and LAZ point cloud data, wrapped in a NITF 2.1 file.

:ref:`writers.null`
    Provides a sink for points in a pipeline. It's the same as sending pipeline
    output to /dev/null.

:ref:`writers.oci`
    Write data to Oracle point cloud databases.

:ref:`writers.ogr`
    Write a point cloud as a set of OGR points/multipoints

:ref:`writers.pcd`
    Write PCD-formatted files in the ASCII, binary, or compressed format.

:ref:`writers.pgpointcloud`
    Write to a PostgreSQL database that has the PostgreSQL Pointcloud extension
    enabled.

:ref:`writers.ply`
    Write points as PLY vertices. Can also emit a mesh as a set of faces.

:ref:`writers.sbet`
    Write data in the SBET format.

:ref:`writers.sqlite`
    Write point cloud data in a scheme that matches the approach used in the
    PostgreSQL Pointcloud and OCI readers.

:ref:`writers.text`
    Write points in a text file. GeoJSON and CSV formats are supported.
