.. _filters.first-in-voxel:

filters.first-in-voxel
===============================================================================

The **first-in-voxel filter** is a voxel-based sampling filter.
The input point
cloud is divided into 3D voxels at the given cell size. For each populated
voxel, the first point entering in the voxel is accepted and voxel is marked as populated. 
All other points entering in the same voxel are ignored/skipped.

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.first-in-voxel",
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
