.. _readers.gdal:

readers.gdal
================================================================================

The `GDAL`_ reader reads `GDAL readable raster`_ data sources as point clouds.

.. _`GDAL`: http://gdal.org
.. _`GDAL readable raster`: http://www.gdal.org/formats_list.html

Each pixel is given an X and Y coordinate (and corresponding PDAL dimensions)
that are center pixel, and each band is represented by "band-1", "band-2", or
"band-n".  Using the 'header' option allows naming the band data to standard
PDAL dimensions.

.. embed::

Basic Example
--------------------------------------------------------------------------------

Simply writing every pixel of a JPEG to a text file is not very useful.

.. code-block:: json

  [
      {
          "type":"readers.gdal",
          "filename":"./pdal/test/data/autzen/autzen.jpg"
      },
      {
          "type":"writers.text",
          "filename":"outputfile.txt"
      }
  ]


LAS Example
--------------------------------------------------------------------------------

The following example assigns the bands from a JPG to the
RGB values of an `ASPRS LAS`_ file using :ref:`writers.las`.

.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html

.. code-block:: json

  [
      {
          "type":"readers.gdal",
          "filename":"./pdal/test/data/autzen/autzen.jpg",
          "header": "Red, Green, Blue"
      },
      {
          "type":"writers.text",
          "filename":"outputfile.txt"
      }
  ]


Options
--------------------------------------------------------------------------------

filename
  `GDALOpen`_ 'able raster file to read [Required]

.. _`GDALOpen`: http://www.gdal.org/gdal_8h.html#a6836f0f810396c5e45622c8ef94624d4

.. include:: reader_opts.rst

header
    A comma-separated list of :ref:`dimension <dimensions>` IDs to map
    bands to. The length of the list must match the number
    of bands in the raster.
