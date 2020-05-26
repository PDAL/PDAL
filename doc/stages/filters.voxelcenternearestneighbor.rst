.. _filters.voxelcenternearestneighbor:

filters.voxelcenternearestneighbor
===============================================================================

The **VoxelCenterNearestNeighbor filter** is a voxel-based sampling filter.
The input point
cloud is divided into 3D voxels at the given cell size. For each populated
voxel, the coordinates of the voxel center are used as the query point in a 3D
nearest neighbor search. The nearest neighbor is then added to the output point
cloud, along with any existing dimensions.

.. note::

    This is similar to the existing :ref:`filters.voxelgrid`. However, in the
    case of the VoxelGrid, the centroid of the points falling within the voxel
    is added to the output point cloud. The drawback with this approach is that
    all dimensional data is lost, and the sampled cloud now consists of only
    XYZ coordinates.

.. embed::


Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.voxelcenternearestneighbor",
          "cell":10.0
      },
      "output.las"
  ]

.. seealso::

    :ref:`filters.voxelcentroidnearestneighbor` offers a similar solution,
    using
    as the query point the centroid of all points falling within the voxel as
    opposed to the voxel center coordinates.

Options
-------------------------------------------------------------------------------

cell
  Cell size in the ``X``, ``Y``, and ``Z`` dimension. [Default: 1.0]
