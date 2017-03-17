.. _readers.gdal:

readers.gdal
================================================================================

The `GDAL`_ reader reads `GDAL readable raster`_ data sources as point clouds.

.. _`GDAL`: http://gdal.org
.. _`GDAL readable raster`: http://www.gdal.org/formats_list.html

Each pixel is given an X and Y coordinate (and corresponding PDAL dimensions)
that are center pixel, and each band is represented by "band-1", "band-2", or
"band-n". The user must know what the bands correspond to, and use
:ref:`filters.ferry` to copy data into known :ref:`dimensions` as needed.


.. note::

    :ref:`filters.ferry` is needed to map GDAL output to typical :ref:`dimensions`
    names. For output to formats such as :ref:`LAS <writers.las>`, this mapping
    is required.


Basic Example
--------------------------------------------------------------------------------

Simply writing every pixel of a JPEG to a text file is not very useful.

.. code-block:: json
    :linenos:

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

The following example assigns the bands from a JPG to the
RGB values of an `ASPRS LAS`_ file using :ref:`writers.las`.

.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html

.. code-block:: json
    :linenos:

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
  `GDALOpen`_ 'able raster file to read [Required]

.. _`GDALOpen`: http://www.gdal.org/gdal_8h.html#a6836f0f810396c5e45622c8ef94624d4


