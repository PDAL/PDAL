.. _filters.miniball:

filters.miniball
===============================================================================

The **Miniball Criterion** was introduced in [Weyrich2004]_ and is based on the
assumption that points that are distant to the cluster built by their
k-neighborhood are likely to be outliers. First, the smallest enclosing ball is
computed for the k-neighborhood, giving a center point and radius
[Fischer2010]_. The miniball criterion is then computed by comparing the
distance (from the current point to the miniball center) to the radius of the
miniball.

The author suggests that the Miniball Criterion is more robust than the
:ref:`Plane Fit Criterion <filters.planefit>` around high-frequency details,
but demonstrates poor outlier detection for points close to a smooth surface.

The filter creates a single new dimension, ``Miniball``, that records the
Miniball criterion for the current point.

.. note::

  To inspect the newly created, non-standard dimensions, be sure to write to an
  output format that can support arbitrary dimensions, such as BPF.

.. embed::

Example
-------

The sample pipeline below computes the Miniball criterion with a neighborhood
of 8 neighbors. We do not apply a fixed threshold to single out outliers based
on the Miniball criterion as the range of values can vary from one dataset to
another. In general, higher values indicate the likelihood of a point being an
outlier.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.miniball",
          "knn":8
      },
      "output.laz"
  ]

Options
-------------------------------------------------------------------------------

knn
  The number of k nearest neighbors. [Default: 8]

.. include:: filter_opts.rst

