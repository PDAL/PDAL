.. _readers.gdal:

readers.gdal
================================================================================

The `GDAL`_ reader reads `GDAL readable raster`_ datasources as point clouds.

.. _`GDAL`: http://gdal.org
.. _`GDAL readable raster`: http://www.gdal.org/formats_list.html

Each pixel is given an X and Y coordinate (and corresponding PDAL dimensions) that
are center pixel, and each band is represented by "band-1", "band-2", or "band-n". The
user must know what the bands correspond to, and use :ref:`filters.ferry` to copy data into
known dimensions as needed.


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
               <Option name="dimension">
                    band-1
                    <Options>
                        <Option name="to">
                            Red
                        </Option>
                    </Options>
                </Option>
               <Option name="dimension">
                    band-2
                    <Options>
                        <Option name="to">
                            Green
                        </Option>
                    </Options>
                </Option>
               <Option name="dimension">
                    band-3
                    <Options>
                        <Option name="to">
                            Blue
                        </Option>
                    </Options>
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


