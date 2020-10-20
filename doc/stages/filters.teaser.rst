.. _filters.teaser:

filters.teaser
==============

The **TEASER filter** uses the Truncated least squares Estimation And
SEmidefinite Relaxation (TEASER) algorithm [Yang2020]_ to calculate a **rigid**
transformation that best aligns two datasets. The first input to the ICP filter
is considered the "fixed" points, and all subsequent points are "moving"
points. The output from the filter are the "moving" points after the calculated
transformation has been applied, one point view per input. The transformation
matrix is inserted into the stage's metadata.

.. seealso::

    The plugin wraps the TEASER++ library, which can be found at
    https://github.com/MIT-SPARK/TEASER-plusplus.

.. plugin::

Examples
--------

.. code-block:: json

  [
      "fixed.las",
      "moving.las",
      {
          "type": "filters.teaser"
      },
      "output.las"
  ]

To get the ``transform`` matrix, you'll need to use the ``--metadata`` option
from the pipeline command:

::

    $ pdal pipeline teaser-pipeline.json --metadata teaser-metadata.json

The metadata output might start something like:

.. code-block:: json

    {
        "stages":
        {
            "filters.teaser":
            {
                "centroid": "    583394  5.2831e+06   498.152",
                "composed": "           1  2.60209e-18 -1.97906e-09       -0.374999  8.9407e-08            1  5.58794e-09      -0.614662 6.98492e -10 -5.58794e-09            1   0.033234           0            0            0            1",
                "converged": true,
                "fitness": 0.01953125097,
                "transform": "           1  2.60209e-18 -1.97906e-09       -0.375  8.9407e-08            1  5.58794e-09      -0.5625 6.98492e -10 -5.58794e-09            1   0.00411987           0            0            0            1"
            }


To apply this transformation to other points, the ``centroid`` and
``transform`` metadata items can by used with ``filters.transformation`` in
another pipeline. First, move the centroid of the points to (0,0,0), then apply
the transform, then move the points back to the original location.  For the
above metadata, the pipeline would be similar to:

.. code-block:: json

    [
        {
            "type": "readers.las",
            "filename": "in.las"
        },
        {
            "type": "filters.transformation",
            "matrix": "1 0 0 -583394   0 1 0 -5.2831e+06   0 0 1 -498.152   0 0 0 1"
        },
        {
            "type": "filters.transformation",
            "matrix": "1  2.60209e-18 -1.97906e-09       -0.375  8.9407e-08            1  5.58794e-09      -0.5625 6.98492e -10 -5.58794e-09            1   0.00411987           0            0            0            1"
        },
        {
            "type": "filters.transformation",
            "matrix": "1 0 0 583394   0 1 0 5.2831e+06  0 0 1 498.152  0 0 0 1"
        },
        {
            "type": "writers.las",
            "filename": "out.las"
        }
    ]

.. note::

    The ``composed`` metadata matrix is a composition of the three transformation steps outlined above, and can be used in a single call to ``filters.transformation`` as opposed to the three separate calls.

.. seealso::

    :ref:`filters.transformation` to apply a transform to other points.


Options
--------

nr
  Radius to use for normal estimation. [Default: **0.02**]

fr
  Radius to use when computing features. [Default: **0.04**]

fpfh
  Use FPFH to find correspondences? [Default: **true**]

.. include:: filter_opts.rst

