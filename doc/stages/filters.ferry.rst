.. _filters.ferry:

filters.ferry
================================================================================

The ferry filter is used to stash intermediate variables as part of
processing data. For example, a common scenario is to keep both the
original value and the reprojected X and Y variables in a
scenario that uses the :ref:`filters.reprojection` filter. In the
normal case, the X and Y data would be overwritten with the new
longitude and latitude values as part of the reprojection. The
ferry filter will allow you to keep this around for later use.


Example
-------

In this scenario, we are doing what is described above --
stashing the pre-projection X and Y values into the
`StatePlaneX` and `StatePlaneY` dimensions. Future
processing, can then operate on these data.

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Writer type="writers.las">
            <Option name="filename">
                colorized.las
            </Option>
            <Filter type="filters.reprojection">
                <Option name="out_srs">
                    EPSG:4326+4326
                </Option>
                <Option name="scale_x">
                    0.0000001
                </Option>
                <Option name="scale_y">
                    0.0000001
                </Option>
                <Filter type="filters.ferry">
                    <Option name="dimensions">
                        X = StatePlaneX, Y=StatePlaneY
                    </Option>
                    <Reader type="readers.las">
                        <Option name="filename">
                            ../las/1.2-with-color.las
                        </Option>
                        <Option name="spatialreference">
                            EPSG:2993
                        </Option>
                    </Reader>
                </Filter>
            </Filter>
        </Writer>
    </Pipeline>

Options
-------

dimensions
  A list of dimensions whose values should be copied to the specified
  dimensions.
  The format of the option is <from>=<to>, <from>=<to>,... Spaces are ignored.
  'from' dimensions must exist and have been created by a reader or filter.
  'to' dimensions will be created if necessary.
