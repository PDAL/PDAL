.. _filters.icp:

filters.icp
==============

The **ICP filter** uses the `PCL's Iterative Closest Point (ICP)`_ algorithm to
calculate a **rigid** (rotation and translation) transformation that best aligns
two datasets.  The first input to the ICP filter is considered the "fixed"
points, and all subsequent points are "moving" points.  The output from the
change filter are the "moving" points after the calculated transformation has
been applied, one point view per input.  The transformation matrix is inserted
into the stage's metadata.

.. note::

    ICP requires that the initial pose of the two point sets to be adequately
    close, which are not always available, especially when transformation is
    non-rigid. ICP can handle limited nonrigid transformations but be aware
    ICP may be unable to escape a local minimum. Consider using CPD instead.

    From :cite:`Xuechen2019`:

    ICP starts with an initial guess of the transformation between the two
    point sets and then iterates between finding the correspondence under the
    current transformation and updating the transformation with the newly
    found correspondence. ICP is widely used because it is rather
    straightforward and easy to implement in practice; however, its biggest
    problem is that it does not guarantee finding the globally optimal
    transformation. In fact, ICP converges within a very small basin in the
    parameter space, and it easily becomes trapped in local minima. Therefore,
    the results of ICP are very sensitive to the initialization, especially
    when high levels of noise and large proportions of outliers exist.


.. plugin::

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

To get the transform matrix, you'll need to use the ``--metadata`` option
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
                "converged": true,
                "fitness": 0.01953125097,
                "transform": "           1  2.60209e-18 -1.97906e-09       -0.375  8.9407e-08            1  5.58794e-09      -0.5625 6.98492e -10 -5.58794e-09            1   0.00411987           0            0            0            1"
            }

.. seealso::

    :ref:`filters.transformation` to apply a transform to other points.
    :ref:`filters.cpd` for the use of a probabilistic assignment of correspondences between pointsets.


Options
--------

None.

.. _PCL's Iterative Closest Point (ICP): http://docs.pointclouds.org/trunk/classpcl_1_1_iterative_closest_point.html
