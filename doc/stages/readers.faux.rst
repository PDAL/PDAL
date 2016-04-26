.. _readers.faux:

readers.faux
============

The "**faux reader**" is used for testing pipelines. It does not read from a
file or database, but generates synthetic data to feed into the pipeline.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.faux",
          "bounds":"([0,1000000],[0,1000000],[0,100])",
          "num_points":"10000",
          "mode":"random"
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }


Options
-------

bounds
  What spatial extent should points be generated within? Text string of the
  form "([xmin,xmax],[ymin,ymax],[zmin,zmax])". [Default: unit cube]

num_points
  How many synthetic points to generate before finishing? [Required]

mean_x|y|z
  Mean value in the x, y, or z dimension respectively. (Normal mode only)
  [Default: 0]

stdev_x|y|z
  Standard deviation in the x, y, or z dimension respectively. (Normal mode
  only) [Default: 1]

mode
  How to generate synthetic points. One of "constant" (repeat single value),
  "random" (random values within bounds), "ramp" (steadily increasing values
  within the bounds), "uniform" (uniformly distributed within bounds), or
  "normal" (normal distribution with given mean and standard deviation).
  [Required]

