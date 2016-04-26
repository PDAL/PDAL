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

.. code-block:: json

    {
      "pipeline":[
        "uncompressed.las",
        {
          "type":"readers.las",
          "spatialreference":"EPSG:2993",
          "filename":"../las/1.2-with-color.las"
        },
        {
          "type":"filters.ferry",
          "dimensions":"X = StatePlaneX, Y=StatePlaneY"
        },
        {
          "type":"filters.reprojection",
          "out_srs":"EPSG:4326+4326"
        },
        {
          "type":"writers.las",
          "scale_x":"0.0000001",
          "scale_y":"0.0000001",
          "filename":"colorized.las"
        }
      ]
    }

Options
-------

dimensions
  A list of dimensions whose values should be copied to the specified
  dimensions.
  The format of the option is <from>=<to>, <from>=<to>,... Spaces are ignored.
  'from' dimensions must exist and have been created by a reader or filter.
  'to' dimensions will be created if necessary.
