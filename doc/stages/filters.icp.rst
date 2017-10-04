.. _filters.icp:

filters.icp
==============

The ICP filter uses the `PCL's Iterative Closest Point (ICP)`_ algorithm to
calculate a rigid (rotation and translation) transformation that best aligns
two datasets.  The first input to the ICP filter is considered the "fixed"
points, and all subsequent points are "moving" points.  The output from the
change filter are the "moving" points after the calculated transformation has
been applied, one point view per input.  The transformation matrix is inserted
into the stage's metadata.

.. plugin::

Examples
--------

.. code-block:: json

    {
        "pipeline": [
            "fixed.las",
            "moving.las",
            {
                "type": "filters.icp"
            },
            "output.las"
        ]
    }

To get the transform matrix, you'll need to use the ``--metadata`` option:

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

Options
--------

None.

.. _PCL's Iterative Closest Point (ICP): http://docs.pointclouds.org/trunk/classpcl_1_1_iterative_closest_point.html
