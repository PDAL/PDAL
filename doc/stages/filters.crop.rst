.. _filters.crop:

filters.crop
============

The crop filter removes points that fall outside or inside a cropping bounding
box (2D)
or polygon.  If more than one bounding region is specified, the filter will
pass all input points through each bounding region, creating an output point
set for each input crop region.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "file-input.las",
        {
          "type":"filters.crop",
          "bounds":"bounds",
          "count":"([0,1000000],[0,1000000])"
        },
        {
          "type":"writers.las",
          "filename":"file-cropped.las"
        }
      ]
    }



Options
-------

bounds
  The extent of the clipping rectangle, expressed in a string, eg: *([xmin, xmax], [ymin, ymax])*  This option can be specified more than once.

polygon
  The clipping polygon, expressed in a well-known text string, eg: *POLYGON((0 0, 5000 10000, 10000 0, 0 0))*  This option can be specified more than once.

outside
  Invert the cropping logic and only take points **outside** the cropping bounds or polygon. [Default: **false**]

