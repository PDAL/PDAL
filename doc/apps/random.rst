.. _random_command:

********************************************************************************
random
********************************************************************************

.. warning::

  As of PDAL v2.6.0, the ``random`` command is marked as DEPRECATED. It will
  be removed from the default install in PDAL v2.7 and removed completely in
  PDAL v2.8.

The ``random`` command is used to create a random point cloud. It uses
:ref:`readers.faux` to create a point cloud containing ``count`` points
drawn randomly from either a uniform or normal distribution. For the uniform
distribution, the bounds can be specified (they default to a unit cube). For
the normal distribution, the mean and standard deviation can both be set for
each of the x, y, and z dimensions.

::

    $ pdal random <output>

::

  --output, -o       Output file name
  --compress, -z     Compress output data (if supported by output format)
  --count            How many points should we write?
  --bounds           Extent (in XYZ to clip output to)
  --mean             A comma-separated or quoted, space-separated list of means
      (normal mode): --mean 0.0,0.0,0.0 --mean "0.0 0.0 0.0"
  --stdev            A comma-separated or quoted, space-separated list of
      standard deviations (normal mode): --stdev 0.0,0.0,0.0 --stdev "0.0 0.0 0.0"
  --distribution     Distribution (uniform / normal)


