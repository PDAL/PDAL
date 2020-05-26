.. _filters.reciprocity:

filters.reciprocity
===============================================================================

The **Nearest-Neighbor Reciprocity Criterion** was introduced in [Weyrich2004]_
and is based on a simple assumption, that valid points may be in the
k-neighborhood of an outlier, but the outlier will most likely not be part of
the valid point's k-neighborhood.

The author suggests that the Nearest-Neighbor Reciprocity Criterion is more
robust than both the :ref:`Plane Fit <filters.planefit>` and :ref:`Miniball
<filters.miniball>` Criterion, being equally sensitive around smooth and
detailed regions. The criterion does however produce invalid reslts near
manifold borders.

The filter creates a single new dimension, ``Reciprocity``, that records the
percentage of points(in the range 0 to 100) that are considered uni-directional
neighbors of the current point. 

.. note::

  To inspect the newly created, non-standard dimensions, be sure to write to an
  output format that can support arbitrary dimensions, such as BPF.

.. embed::

Example
-------

The sample pipeline below computes reciprocity with a neighborhood of 8
neighbors, followed by a range filter to crop out points whose ``Reciprocity``
percentage is less than 98% before writing the output.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.reciprocity",
          "knn":8
      },
      {
          "type":"filters.range",
          "limits":"Reciprocity[:98.0]"
      },
      "output.laz"
  ]

Options
-------------------------------------------------------------------------------

knn
  The number of k nearest neighbors. [Default: 8]

