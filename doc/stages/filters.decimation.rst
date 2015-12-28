.. _filters.decimation:

filters.decimation
==================

The decimation filter retains every Nth point from an input point view.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">smaller.las</Option>
      <Filter type="filters.decimation">
        <Option name="step">10</Option>
        <Reader type="readers.las">
            <Option name="filename">larger.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

.. seealso::

    :ref:`filters.voxelgrid` provides grid-style point decimation.

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

