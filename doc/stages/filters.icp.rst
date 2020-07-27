.. _filters.icp:

filters.icp
==============

The **ICP filter** uses the Iterative Closest Point (ICP) algorithm to
calculate a **rigid** (rotation and translation) transformation that best
aligns two datasets.  The first input to the ICP filter is considered the
"fixed" points, and all subsequent points are "moving" points.  The output from
the filter are the "moving" points after the calculated transformation has been
applied, one point view per input.  The transformation matrix is inserted into
the stage's metadata.

.. note::

    ICP requires the initial pose of the two point sets to be adequately close,
    which is not always possible, especially when the transformation is
    non-rigid.  ICP can handle limited non-rigid transformations but be aware
    ICP may be unable to escape a local minimum. Consider using CPD instead.

    From :cite:`Xuechen2019`:

    ICP starts with an initial guess of the transformation between the two
    point sets and then iterates between finding the correspondence under the
    current transformation and updating the transformation with the newly found
    correspondence. ICP is widely used because it is rather straightforward and
    easy to implement in practice; however, its biggest problem is that it does
    not guarantee finding the globally optimal transformation. In fact, ICP
    converges within a very small basin in the parameter space, and it easily
    becomes trapped in local minima. Therefore, the results of ICP are very
    sensitive to the initialization, especially when high levels of noise and
    large proportions of outliers exist.


Examples
--------

.. code-block:: json

  [
      "fixed.las",
      "moving.las",
      {
          "type": "filters.icp"
      },
      "output.las"
  ]

To get the ``transform`` matrix, you'll need to use the ``--metadata`` option
from the pipeline command:

::

    $ pdal pipeline icp-pipeline.json --metadata icp-metadata.json

The metadata output might start something like:

.. code-block:: json

    {
        "stages":
        {
            "filters.icp":
            {
                "centroid": "    583394  5.2831e+06   498.152",
                "composed": "           1  2.60209e-18 -1.97906e-09       -0.374999  8.9407e-08            1  5.58794e-09      -0.614662 6.98492e -10 -5.58794e-09            1   0.033234           0            0            0            1",
                "converged": true,
                "fitness": 0.01953125097,
                "transform": "           1  2.60209e-18 -1.97906e-09       -0.375  8.9407e-08            1  5.58794e-09      -0.5625 6.98492e -10 -5.58794e-09            1   0.00411987           0            0            0            1"
            }


To apply this transformation to other points, the ``centroid`` and ``transform``
metadata items can by used with ``filters.transformation`` in another pipeline.  First,
move the centroid of the points to (0,0,0), then apply the transform, then move
the points back to the original location.  For the above metadata, the pipeline
would be similar to:

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
    :ref:`filters.cpd` for the use of a probabilistic assignment of correspondences between pointsets.


Options
--------

max_iter
  Maximum number of iterations. [Default: **100**]

max_similar
  Max number of similar transforms to consider converged. [Default: **0**]

mse_abs
  Absolute threshold for MSE. [Default: **1e-12**]

rt
  Rotation threshold. [Default: **0.99999**]

tt
  Translation threshold. [Default: **9e-8**]

.. include:: filter_opts.rst

