.. _filters.voxeldownsize:

filters.voxeldownsize
===============================================================================

The **voxeldownsize filter** is a voxel-based sampling filter.
The input point cloud is divided into 3D voxels at the given cell size.
For each populated voxel, either first point entering in the voxel or
center of a voxel (depending on mode argument) is accepted and voxel is
marked as populated.  All other points entering in the same voxel are
filtered out.

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.voxeldownsize",
          "cell":1.0,
          "mode":"center"
      },
      "output.las"
  ]

.. seealso::

    :ref:`filters.voxelcenternearestneighbor` offers a similar solution,
    using the coordinates of the voxel center as the query point in a 3D
    nearest neighbor search.  The nearest neighbor is then added to the
    output point cloud, along with any existing dimensions.

Options
-------------------------------------------------------------------------------

cell
  Cell size in the ``X``, ``Y``, and ``Z`` dimension. [Default: 0.001]

mode
  Mode for voxel based filtering. [Default: center]
  **center**: Coordinates of the first point found in each voxel will
  be modified to be the center of the voxel.
  **first**: Only the first point found in each voxel is retained.

.. warning::
    If you choose **center** mode, you are overwriting the X, Y and Z
    values of retained points.  This may invalidate other dimensions of
    the point if they depend on this location or the location of other points
    in the input.
  
