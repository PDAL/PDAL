.. _filters.dartthrowing:

===============================================================================
filters.dartthrowing
===============================================================================

The practice of performing Poisson sampling via "Dart Throwing" was introduced in the mid-1980's by [Cook1986]_ and [Dippe1985]_, and has been applied to point clouds in other software [Mesh2009]_. Our implementation is a brute force approach that randomly selects points from the input ``PointView``, adding them to the output ``PointView`` subject to the minimum distance constraint (the ``radius``). The full layout (i.e., the dimensions) of the input ``PointView`` is kept in tact (the same cannot be said for :ref:`filters.voxelgrid`).

.. seealso::

    :ref:`filters.decimation` and :ref:`filters.voxelgrid` also perform
    decimation.

.. [Cook1986] Cook, Robert L. "Stochastic sampling in computer graphics." *ACM Transactions on Graphics (TOG)* 5.1 (1986): 51-72.

.. [Dippe1985] Dippé, Mark AZ, and Erling Henry Wold. "Antialiasing through stochastic sampling." *ACM Siggraph Computer Graphics* 19.3 (1985): 69-78.

.. [Mesh2009] ALoopingIcon. "Meshing Point Clouds." *MESHLAB STUFF*. n.p., 7 Sept. 2009. Web. 13 Nov. 2015.

Options
-------------------------------------------------------------------------------

radius
  Minimum distance between samples. [Default: **1.0**]
