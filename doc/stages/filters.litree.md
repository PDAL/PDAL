.. _filters.litree:

===============================================================================
filters.litree
===============================================================================

The purpose of the Li tree filter is to segment individual trees from an input
``PointView``. In the output ``PointView`` points that are deemed to be part of
a tree are labeled with a ``ClusterID``. Tree IDs start at 1, with non-tree points
given a ``ClusterID`` of 0.

.. note::

  The filter differs only slightly from the paper in the addition of a few
  conditions on size of tree, minimum height above ground for tree seeding, and
  flexible radius for non-tree seed insertion.

.. note::

  In earlier PDAL releases (up to v2.2.0), ``ClusterID`` was stored in the
  ``TreeID`` Dimemsion.

.. embed::

Example
-------

The Li tree algorithm expects to visit points in descending order of
``HeightAboveGround``, which is also used in determining the minimum tree
height to consider. As such, the following pipeline precomputes
``HeightAboveGround`` using :ref:`filters.hag_delaunay` and subsequently sorts
the ``PointView`` using this dimension.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.hag_delaunay"
      },
      {
          "type":"filters.sort",
          "dimension":"HeightAboveGround",
          "order":"DESC"
      },
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

