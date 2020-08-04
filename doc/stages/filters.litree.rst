.. _filters.litree:

===============================================================================
filters.litree
===============================================================================

The purpose of the Li tree filter is to segment individual trees from an input
``PointView``. In the output ``PointView`` points that are deemed to be part of
a tree are labeled with a ``TreeID``. Tree IDs start at 1, with non-tree points
given a ``TreeID`` of 0.

.. note::

  The filter differs only slightly from the paper in the addition of a few
  conditions on size of tree, minimum height above ground for tree seeding, and
  flexible radius for non-tree seed insertion.

.. embed::

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.litree",
          "min_points":50,
          "min_height":10.0,
          "radius":200.0
      },
      {
          "type":"writers.las",
          "filename":"output.laz",
          "minor_version":1.4,
          "extra_dims":"all"
      }
  ]

Options
-------

min_points
  Minimum number of points in a tree cluster. [Default: 10]

min_height
  Minimum height above ground to start a tree cluster. [Default: 3.0]

radius
  The seed point for the non-tree cluster is the farthest point in a 2D
  Euclidean sense from the seed point for the current tree. [Default: 100.0]

.. include:: filter_opts.rst

