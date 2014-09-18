.. _filters.decimation:

filters.decimation
==================

The decimation filter takes a stream of points and steps through it, taking only every Nth point. With a step of 2, the filter takes every second point, for a reduction of 50%. With a step of 10, the filter takes every tenth point, for a reduction of 90%.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">smaller.las</Option>
      <Filter type="filters.decimation">
        <Option name="step">10</Option>
        <Reader type="drivers.las.reader">
            <Option name="filename">larger.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

Options
-------

step
  How many points to skip between each sample point? A step of 1 will skip no point, and leave the stream unchanged. A step of 100 will reduce the stream by 99%. [Default: **1**]
  
offset
  Start sampling with what point? [Default: **0**]
