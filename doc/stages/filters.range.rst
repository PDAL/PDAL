.. _filters.range:

filters.range
======================

The range filter applies rudimentary filtering to the input point cloud
based on a set of criteria on the given dimensions.

Example
-------

This example passes through all points whose Z value is in the range [0,100]
and whose classification equals 2 (corresponding to ground in LAS).

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        filtered.las
      </Option>
      <Filter type="filters.range">
        <Option name="dimension">
          Z
          <Options>
            <Option name="min">0</Option>
            <Option name="max">100</Option>
          </Options>
        </Option>
        <Option name="dimension">
          Classification
          <Options>
            <Option name="equals">2</Option>
          </Options>
        </Option>
        <Reader type="readers.las">
          <Option name="filename">
            input.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>


Options
-------

dimension
  A dimension Option with an Options block containing the range criteria.

  * min: The minimum allowable value (inclusive) to pass through to the filtered point cloud.

  * max: The maximum allowable value (inclusive) to pass through to the filtered point cloud.

  * equals: The exact value to pass through to the filtered point cloud (i.e., min = max).

