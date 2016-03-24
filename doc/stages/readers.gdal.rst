.. _readers.gdal:

readers.gdal
================================================================================

The `GDAL`_ reader reads `GDAL readable raster`_ data sources as point clouds.

.. _`GDAL`: http://gdal.org
.. _`GDAL readable raster`: http://www.gdal.org/formats_list.html

Each pixel is given an X and Y coordinate (and corresponding PDAL dimensions)
that are center pixel, and each band is represented by "band-1", "band-2", or
"band-n". The user must know what the bands correspond to, and use
:ref:`filters.ferry` to copy data into known dimensions as needed.


.. note::

    :ref:`filters.ferry` is needed because raster data do not map to
    typical dimension names. For output to formats such as :ref:`LAS <writers.las>`,
    this mapping is required.


Basic Example
--------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.gdal",
          "filename":"./pdal/test/data/autzen/autzen.jpg"
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }



LAS Example
--------------------------------------------------------------------------------

The following example writes a JPG as an `ASPRS LAS`_ file.

.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.gdal",
          "filename":"./pdal/test/data/autzen/autzen.jpg"
        },
        {
          "type":"filters.ferry"
          "dimensions":"band-1=Red, band-2=Green, band-3=Blue",
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }



Options
--------------------------------------------------------------------------------

filename
  GDALOpen'able raster file to read [Required]


