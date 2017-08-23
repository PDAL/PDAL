.. _filters.crop:

filters.crop
============

The crop filter removes points that fall outside or inside a cropping bounding
box (2D), polygon, or point+distance.  If more than one bounding region is
specified, the filter will pass all input points through each bounding region,
creating an output point set for each input crop region.

.. embed::

.. streamable::

The provided bounding regions are assumed to have the same spatial reference
as the points unless the option `a_srs` provides an explicit spatial reference
for bounding regions.
If the point input consists of multiple point views with differing
spatial references, one is chosen at random and assumed to be the
spatial reference of the input bounding region.  In this case a warning will
be logged.


Example
-------

.. code-block:: json

    {
      "pipeline":[
        "file-input.las",
        {
          "type":"filters.crop",
          "bounds":"([0,1000000],[0,1000000])"
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

point
  An array of WKT or GeoJSON 2D or 3D points. Requires ``distance``.

distance
  Distance in units of common X, Y, and Z :ref:`dimensions` to crop circle or sphere in combination with ``point``.

a_srs
  Indicates the spatial reference of the bounding regions.  If not provided,
  it is assumed that the spatial reference of the bounding region matches
  that of the points (if possible).
