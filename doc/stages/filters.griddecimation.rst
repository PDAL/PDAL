.. _filters.crop:

filters.crop
============

The **grid decimation filter** keeps only one point in a grid. The point selected could be the highest or the lowest point. It can be used to quickly filter vegetation points in order to keep only the canopy points.

.. embed::

Example
-------
This example transform highest points of classification 5 in classification 9, on a grid of 0.75m square.

.. code-block:: json

  [
     "file-input.las",
    {
        "type": "filters.gridDecimation",
	"output_type":"max",
        "resolution": "0.75",
	"where":"Classification==5",
	"value":"Classification=9"
    },
    {
          "type":"writers.las",
          "filename":"file-output.las"
    }
  ]


Options
-------

output_type
  The type of points kept. The value should be ``"max"`` for kept the highest point, or ``"min"`` for the lowest.

resolution
  The resolution of the cells in meter.

outside
  Invert the cropping logic and only take points outside the cropping
  bounds or polygon. [Default: false]

.. include:: filter_opts.rst