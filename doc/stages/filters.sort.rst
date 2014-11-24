.. _filters.sort:

filters.sort
============

The sort filter orders a point buffer based on the values of a dimension.
The current filter only supports sorting based on a single dimension in
increasing order.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        sorted.las
      </Option>
      <Filter type="filters.sort">
        <Option name="dimension">
          X
        </Option>
        <Reader type="readers.las">
          <Option name="filename">
            unsorted.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>


Options
-------

dimension
  The dimension on which to sort the points.

Notes
-----

The sorting algorithm used is not stable, meaning that one cannot chain
multiple Sort filters to sort order point buffer heirarchically (say,
primarily by the dimension X and secondairly by the dimension Y).
