.. _filters.planefit:

filters.planefit
===============================================================================

The **Plane Fit Criterion** was introduced in [Weyrich2004]_ and computes the
deviation of a point from a manifold approximating its neighbors.  First, a
plane is fit to each point's k-neighborhood by performing an eigenvalue
decomposition. Next, the mean point to plane distance is computed by
considering all points within the neighborhood. This is compared to the point
to plane distance of the current point giving rise to the k-neighborhood. As
the mean distance of the k-neighborhood approaches 0, the Plane Fit criterion
will tend toward 1. As point to plane distance of the current point approaches
0, the Plane Fit criterion will tend toward 0.

The author suggests that the Plane Fit Criterion is well suited to outlier
detection when considering noisy reconstructions of smooth surfaces, but
produces poor results around small features and creases.

The filter creates a single new dimension, ``PlaneFit``, that records the
Plane Fit criterion for the current point.

.. note::

  To inspect the newly created, non-standard dimensions, be sure to write to an
  output format that can support arbitrary dimensions, such as BPF.

.. embed::

Example
-------

The sample pipeline below computes the Plane Fit criterion with a neighborhood
of 8 neighbors. We do not apply a fixed threshold to single out outliers based
on the Plane Fit criterion as the range of values can vary from one dataset to
another. In general, higher values indicate the likelihood of a point being an
outlier.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.planefit",
          "knn":8
      },
      "output.laz"
  ]

Options
-------------------------------------------------------------------------------

knn
  The number of k nearest neighbors. [Default: 8]

threads
  The number of threads used for computing the plane fit criterion. [Default: 1]

.. include:: filter_opts.rst

