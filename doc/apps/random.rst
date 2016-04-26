.. _random_command:

********************************************************************************
random
********************************************************************************

The ``random`` command is used to create a random point cloud. It uses
:ref:`readers.faux` to create a point cloud containing ``count`` points
drawn randomly from either a uniform or normal distribution. For the uniform
distribution, the bounds can be specified (they default to a unit cube). For
the normal distribution, the mean and standard deviation can both be set for
each of the x, y, and z dimensions.

::

    $ pdal random <output>

::

    --output [-o] arg   Non-positional argument to specify output file name.
    --compress [-z]     Compress output data (if supported by output format)
    --count arg         Number of points in created point cloud [0].
    --bounds arg        Extent (in XYZ to clip output to):
                        --bounds "([xmin,xmax],[ymin,ymax],[zmin,zmax])"
    --mean arg          List of means (for --distribution normal)
                        --mean 0.0,0.0,0.0
                        --mean "0.0 0.0 0.0"
    --stdev arg         List of standard deviations (for --distribution normal)
                        --stdev 0.0,0.0,0.0
                        --stdev "0.0 0.0 0.0"
    --distribution arg  Distribution type (uniform or normal) [uniform]



