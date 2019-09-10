.. _filters.streamablevoxeldownsize:

filters.streamablevoxeldownsize
===============================================================================

The **streamablevoxeldownsize filter** is a voxel-based sampling filter.
The input point
cloud is divided into 3D voxels at the given cell size. For each populated
voxel, either first point entering in the voxel or center of a voxel (depending on mode argument) is accepted and voxel is marked as populated. 
All other points entering in the same voxel are ignored/skipped.

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.streamablevoxeldownsize",
          "cell":1.0,
          "mode":"voxelcenter"
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
  Cell size in the ``X``, ``Y``, and ``Z`` dimension. [Default: 0.001]

mode
  Mode for voxel based filtering. [Default: voxelcenter]
  **voxelcenter**: Point coordinates will be modified and set to the center of a populated voxel.
  **firstinvoxel**: Retain first point detected in each voxel.
