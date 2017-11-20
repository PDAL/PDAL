.. _writers.ogr:

writers.ogr
===========

The **OGR Writer** will create files of various `vector formats`_ as supported
by the OGR library.  PDAL points are generally stored as points in the
output format, though PDAL will create multipoint objects instead of point
objects if the 'multicount' argument is set to a value greater than 1.
Points can be written with a single additional value in addition to location
if 'measure_dim' specifies a valid PDAL dimension and the output format
supports measure point types.

By default, the OGR writer will create ESRI shapefiles.  The particular OGR
driver can be specified with the 'ogrdriver' option.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "inputfile.las",
        {
          "type": "writers.ogr",
          "filename" "outfile.geojson",
          "measure_dim": "Compression"
        }
      ]
    }

Options
-------

filename
  Output file to write.  The writer will accept a filename containing
  a single placeholder character (`#`).  If input to the writer consists
  of multiple PointViews, each will be written to a separate file, where
  the placeholder will be replaced with an incrementing integer.  If no
  placeholder is found, all PointViews provided to the writer are
  aggregated into a single file for output.  Multiple PointViews are usually
  the result of multiple input files, or using :ref:`filters.splitter`,
  :ref:`filters.chipper` or :ref:`filters.divider`.

  The driver will use the OGR GEOjson driver if the output filename
  extension is 'geojson', and the ESRI shapefile driver if the output
  filename extension is 'shp'.
  If neither extension is recognized, the filename is taken
  to represent a directory in which ESRI shapefiles are written.  The
  driver can be explicitly specified by using the 'ogrdriver' option.

multicount
  If 1, point objects will be written.  If greater than 1, specifies the
  number of points to group into a multipoint object.  Not all OGR
  drivers support multipoint objects. [Default: 1]

measure_dim
  If specified, points will be written with an extra data field, the dimension
  of which is specified by this option. Not all output formats support
  measure data. [Default: None]

  .. note::

    The **measure_dim** option is only supported if PDAL is built with
    GDAL version 2.1 or later.

ogrdriver
  The OGR driver to use for output.  This option overrides any inference made
  about output drivers from 'filename'.


.. _vector formats: http://www.gdal.org/ogr_formats.html

