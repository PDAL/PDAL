.. _filters.relaxationdartthrowing:

filters.relaxationdartthrowing
===============================================================================

The **Relaxation Dart Throwing Filter** is a variation on Poisson sampling. The
approach was first introduced by [McCool1992]_. The filter operates nearly
identically to :ref:`filters.sample`, except it will continue to shrink the
radius with each pass through the point cloud until the desired number of
output points is reached.

.. seealso::

    :ref:`filters.decimation`, :ref:`filters.fps` and :ref:`filters.sample` all
    perform some form of thinning or resampling.

.. note::

    The ``shuffle`` option does not reorder points in the PointView, but
    shuffles the order in which the points are visited while processing, which
    can improve the quality of the result.

.. embed::

Options
-------------------------------------------------------------------------------

decay
  Decay rate for the radius shrinkage. [Default: 0.9]

radius
  Starting minimum distance between samples. [Default: 1.0]

count
  Desired number of points in the output. [Default: 1000]

shuffle
  Choose whether or not to shuffle order in which points are visited. [Default:
  true]

seed
  Seed for random number generator, used only with shuffle.

.. include:: filter_opts.rst

