.. _filters.decimation:

filters.decimation
==================

The **decimation filter** retains every Nth point from an input point view.

.. embed::

.. streamable::

Example
-------

.. code-block:: json

  [
      {
          "type": "readers.las",
          "filename": "larger.las"
      },
      {
          "type":"filters.decimation",
          "step": 10
      },
      {
          "type":"writers.las",
          "filename":"smaller.las"
      }
  ]

Options
-------

step
  Number of points to skip between each sample point.  A step of 1 will skip
  no points.  A step of 2 will skip every other point.  A step of 100 will
  reduce the input by ~99%. [Default: 1]

offset
  Point index to start sampling.  Point indexes start at 0.  [Default: 0]

limit
  Point index at which sampling should stop (exclusive).  [Default: No limit]

.. include:: filter_opts.rst

