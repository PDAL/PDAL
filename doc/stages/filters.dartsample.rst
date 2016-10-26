.. _filters.dartsample:

===============================================================================
filters.dartsample
===============================================================================

The practice of performing Poisson sampling via "Dart Throwing" was introduced
in the mid-1980's by [Cook1986]_ and [Dippe1985]_, and has been applied to
point clouds in other software [Mesh2009]_. Our implementation is a brute force
approach that randomly selects points from the input ``PointView``, adding them
to the output ``PointView`` subject to the minimum distance constraint (the
``radius``). The full layout (i.e., the dimensions) of the input ``PointView``
is kept in tact (the same cannot be said for :ref:`filters.voxelgrid`).

.. seealso::

    :ref:`filters.decimation` and :ref:`filters.voxelgrid` also perform
    decimation.


Options
-------------------------------------------------------------------------------

radius
  Minimum distance between samples. [Default: **1.0**]
