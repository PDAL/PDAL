.. _readers.faux:

readers.faux
============

The faux reader is used for testing pipelines. It does not read from a
file or database, but generates synthetic data to feed into the pipeline.

The faux reader requires a mode argument to define the method in which points
should be generated.  Valid modes are as follows:

constant
    The values provided as the minimums to the bounds argument are
    used for the X, Y and Z value, respectively, for every point.
random
    Random values are chosen within the provided bounds.
ramp
    Value increase uniformly from the minimum values to the maximum values.
uniform
    Random values of each dimension are uniformly distributed in the
    provided ranges.
normal
    Random values of each dimension are normally distributed in the
    provided ranges.
grid
    Creates points with integer-valued coordinates in the range provided
    (excluding the upper bound).

.. embed::

.. streamable::

Example
-------

.. code-block:: json

  [
      {
          "type":"readers.faux",
          "bounds":"([0,1000000],[0,1000000],[0,100])",
          "count":"10000",
          "mode":"random"
      },
      {
          "type":"writers.text",
          "filename":"outputfile.txt"
      }
  ]


Options
-------

bounds
  The spatial extent within which points should be generated.
  Specified as a string in the form "([xmin,xmax],[ymin,ymax],[zmin,zmax])".
  [Default: unit cube]

count
  The number of points to generate. [Required, except when mode is 'grid']

override_srs
  Spatial reference to apply to data. [Optional]

mean_x|y|z
  Mean value in the x, y, or z dimension respectively. (Normal mode only)
  [Default: 0]

stdev_x|y|z
  Standard deviation in the x, y, or z dimension respectively. (Normal mode
  only) [Default: 1]

mode
  "constant", "random", "ramp", "uniform", "normal" or "grid" [Required]

