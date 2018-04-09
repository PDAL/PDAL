.. _filters.ferry:

filters.ferry
================================================================================

The ferry filter copies data from one dimension to another, creates new
dimensions or both.

The filter is guided by a list of 'from' and 'to' dimensions in the format
<from>=><to>.  Data from the 'from' dimension is copied to the 'to' dimension.
The 'from' dimension must exist.  The 'to' dimension can be pre-existing or
will be created by the ferry filter.

Alternatively, the format =><to> can be used to create a new dimension without
copying data from any source.  The values of the 'to' dimension are default
initialized.

.. embed::

.. streamable::

Example 1
---------

In this scenario, we are making copies of the X and Y dimensions into the
dimensions StatePlaneX and StatePlaneY.  Since the reprojection filter will
modify the dimensions X and Y, this allows us to maintain both the
pre-reprojection values and the post-reprojection values.


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
          "dimensions":"X => StatePlaneX, Y=>StatePlaneY"
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

Example 2
---------

The ferry filter is being used to add a dimension 'Classification' to points
so that the value can be set to '2' and written as a LAS file.

.. code-block:: json

    {
      "pipeline":[
        {
            "type": "readers.gdal",
            "filename": "somefile.tif"
        },
        {
            "type": "filters.ferry",
            "dimensions": "=>Classification"
        },
        {
            "type": "filters.assign",
            "assignment": "Classification[:]=2"
        },
        "out.las"
      ]
    }

Options
-------

dimensions
  A list of dimensions whose values should be copied.
  The format of the option is <from>=><to>, <from>=><to>,...
  Spaces are ignored.
  'from' can be left empty, in which case the 'to' dimension is created and
  default-initialized.  'to' dimensions will be created if necessary.

  Note: the old syntax that used '=' instead of '=>' between dimension names
  is still supported.
