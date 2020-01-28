.. _filters.hag_dem:

filters.hag_dem
===============================================================================

 loads a GDAL readable raster
 specifying the DEM to calculate ``HeightAboveGround`` from. The ``Z`` value
 of each point is compared against the value at the corresponding X,Y point in
 the DEM raster.

 If `respect_ground_classification`_ is set, any points classified as ground will
 have their ``HeightAboveGround`` value set to 0 regardless of the raster.

 Any points that do not have a corresponding value in the raster DEM will not have
 a ``HeightAboveGround`` value set.

The **Height Above Ground (HAG) Digital Elevation Model (DEM) filter** loads
a GDAL-readable raster image specifying the DEM. The ``Z`` value of each point
in the input is compared against the value at the corresponding X,Y location
in the DEM raster. It creates a new dimension, ``HeightAboveGround``, that
contains the normalized height values.

Normalized heights are a commonly used attribute of point cloud data. This can
also be referred to as *height above ground* (HAG) or *above ground level* (AGL)
heights. In the end, it is simply a measure of a point's relative height as
opposed to its raw elevation value.

.. embed::

Example #1
----------

Using the autzen dataset (here shown colored by elevation)

.. image:: ./images/autzen-elevation.png
   :height: 400px

and a DEM generated from it

::
  
  # pdal translate autzen.laz autzen-dem.tiff filters.smrf

we execute the following pipeline

.. code-block:: json

  [
      "autzen.laz",
      {
          "type":"filters.hag_dem",
          "raster": "autzen-dem.tiff"
      },
      {
          "type":"writers.bpf",
          "filename":"autzen-height.bpf",
          "output_dims":"X,Y,Z,HeightAboveGround"
      }
  ]

which is equivalent to the ``pdal translate`` command

::

    $ pdal translate autzen.laz autzen-height.bpf hag_dem \
        --filters.hag_dem.raster=autzen-dem.tiff \
        --writers.bpf.output_dims="X,Y,Z,HeightAboveGround"

In either case, the result, when colored by the normalized height instead of
elevation is

.. image:: ./images/autzen-height.png
   :height: 400px

Options
-------------------------------------------------------------------------------

_`raster`
    GDAL-readable raster to use for DEM.

band
    GDAL Band number to read (count from 1).
    [Default: 1]

zero_ground
    If true, set HAG of ground-classified points to 0 rather than comparing
    ``Z`` value to raster DEM.
    [Default: true]

