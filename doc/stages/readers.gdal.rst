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

.. code-block:: xml

    <?xml version="1.0"?>
    <Pipeline version="1.0">
        <Writer type="writers.text">
            <Option name="filename">
                out.asc
            </Option>
            <Reader type="readers.gdal">
                <Option name="filename">
                    /Users/hobu/dev/git/pdal/test/data/autzen/autzen.jpg
                </Option>
            </Reader>
        </Writer>
    </Pipeline>

LAS Example
--------------------------------------------------------------------------------

The following example writes a JPG as an `ASPRS LAS`_ file.

.. _`ASPRS LAS`: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html

.. code-block:: xml

    <?xml version="1.0"?>
    <Pipeline version="1.0">
        <Writer type="writers.las">
            <Option name="filename">
                out.las
            </Option>
            <Filter type="filters.ferry">
               <Option name="dimensions">
                    Red=band-1, Green=band-2, Blue=band-3
                </Option>
            <Reader type="readers.gdal">
                <Option name="filename">
                    /Users/hobu/dev/git/pdal/test/data/autzen/autzen.jpg
                </Option>
            </Reader>
        </Filter>
        </Writer>
    </Pipeline>


Options
--------------------------------------------------------------------------------

filename
  GDALOpen'able raster file to read [Required]


