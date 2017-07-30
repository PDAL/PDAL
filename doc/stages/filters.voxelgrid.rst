.. _filters.voxelgrid:

===============================================================================
filters.voxelgrid
===============================================================================

The Voxel Grid filter passes data through the Point Cloud Library (`PCL`_)
VoxelGrid algorithm.

VoxelGrid assembles a local 3D grid over a given PointCloud, and downsamples +
filters the data. The VoxelGrid class creates a *3D voxel grid* (think about a
voxel grid as a set of tiny 3D boxes in space) over the input point cloud data.
Then, in each *voxel* (i.e., 3D box), all the points present will be
approximated (i.e., *downsampled*) with their centroid. This approach is a bit
slower than approximating them with the center of the voxel, but it represents
the underlying surface more accurately.

.. embed::

Example
-------


.. code-block:: json

    {
      "pipeline":[
        "untransformed.las",
        {
          "type":"filters.voxelgrid"
        },
        {
          "type":"writers.las",
          "filename":"transformed.las"
        }
      ]
    }


.. seealso::

    :ref:`filters.decimation` does simple every-other-X -style decimation.

.. _`PCL`: http://www.pointclouds.org

Options
-------------------------------------------------------------------------------

leaf_x
  Leaf size in X dimension. [Default: **1.0**]

leaf_y
  Leaf size in Y dimension. [Default: **1.0**]

leaf_z
  Leaf size in Z dimension. [Default: **1.0**]
