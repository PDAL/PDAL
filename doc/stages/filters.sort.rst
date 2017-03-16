.. _filters.sort:

filters.sort
============

The sort filter orders a point view based on the values of a dimension. The
sorting can be done in increasing (ascending) or decreasing (descending) order.

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
        <Option name="order">
          ASC
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

order
  The order in which to sort, ASC or DESC [Default: **ASC**]
