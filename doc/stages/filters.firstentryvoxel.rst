.. _filters.firstentryvoxel:

filters.firstentryvoxel
===============================================================================

The **firstentryvoxel filter** is a voxel-based sampling filter.
The input point
cloud is divided into 3D voxels at the given cell size. For each populated
voxel, the first point fitting in the voxel is accepted and all others are ignored.

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.firstentryvoxel",
          "cell":1.0
      },
      "output.las"
  ]

.. seealso::

    :ref:`filters.voxelcenternearestneighbor` offers a similar solution,
    using
    the coordinates of the voxel center as the query point in a 3D nearest neighbor search. 
    The nearest neighbor is then added to the output point cloud, along with any existing dimensions.

Options
-------------------------------------------------------------------------------

cell
  Cell size in the ``X``, ``Y``, and ``Z`` dimension.
