.. _filters.farthestpointsampling:

filters.farthestpointsampling
===============================================================================

The **Farthest Point Sampling Filter** adds points from the input to the output
``PointView`` one at a time by selecting the point from the input cloud that is
farthest from any point currently in the output.



.. seealso::

    :ref:`filters.sample` produces a similar result, but while
    ``filters.sample`` allows us to target a desired separation of points via
    the ``radius`` parameter at the expense of knowing the number of points in
    the output, ``filters.farthestpointsampling`` allows us to specify exactly
    the number of output points at the expense of knowing beforehand the
    spacing between points.

.. embed::

Options
-------------------------------------------------------------------------------

count
  Desired number of output samples. [Default: 1000]

.. include:: filter_opts.rst

