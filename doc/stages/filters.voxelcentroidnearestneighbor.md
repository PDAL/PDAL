.. _filters.voxelcentroidnearestneighbor:

filters.voxelcentroidnearestneighbor
===============================================================================

The **VoxelCentroidNearestNeighbor Filter** is a voxel-based sampling filter.
The input point cloud is divided into 3D voxels at the given cell size. For
each populated voxel, we apply the following ruleset. For voxels with only one
point, the point is passed through to the output. For voxels with exactly two
points, the point closest the voxel center is returned. Finally, for voxels
with more than two points, the centroid of the points within that voxel is
computed. This centroid is used as the query point in a 3D nearest neighbor
search (considering only those points lying within the voxel). The nearest
neighbor is then added to the output point cloud, along with any existing
dimensions.

.. embed::

Example
-------


.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.voxelcentroidnearestneighbor",
          "cell":10.0
      },
      "output.las"
  ]

.. seealso::

    :ref:`filters.voxelcenternearestneighbor` offers a similar solution, using
    the voxel center as opposed to the voxel centroid for the query point.

Options
-------------------------------------------------------------------------------

cell
  Cell size in the ``X``, ``Y``, and ``Z`` dimension. [Default: 1.0]

.. include:: filter_opts.rst

