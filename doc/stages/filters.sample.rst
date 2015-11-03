.. _filters.sample:

filters.sample
===============================================================================

The **Sample Filter** performs Poisson sampling of the input ``PointView``. The
The practice of performing Poisson sampling via "Dart Throwing" was introduced
in the mid-1980's by [Cook1986]_ and [Dippe1985]_, and has been applied to
point clouds in other software [Mesh2009]_.

The sampling can be performed in a single pass through the point cloud.
To begin,
each input point is assumed to be kept. As we iterate through the kept points,
we retrieve all neighbors within a given ``radius``, and mark these neighbors as
points to be discarded. All remaining kept points are appended to the output
``PointView``. The full layout (i.e., the dimensions) of the input ``PointView``
is kept in tact (the same cannot be said for :ref:`filters.voxelgrid`).

.. seealso::

    :ref:`filters.decimation`, :ref:`filters.farthestpointsampling` and
    :ref:`filters.relaxationdartthrowing` also perform decimation.

.. note::

    The ``shuffle`` option does not reorder points in the PointView, but
    shuffles the order in which the points are visited while processing, which
    can improve the quality of the result.

.. embed::

Options
-------------------------------------------------------------------------------

radius
  Minimum distance between samples. [Default: 1.0]

shuffle
  Choose whether or not to shuffle order in which points are visited. [Default:
  true]

seed
  Seed for random number generator, used only with shuffle.

.. include:: filter_opts.rst

