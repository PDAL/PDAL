.. _filters.mortonorder:

filters.mortonorder
===========

Sorts the XY data using `Morton ordering`_.

.. _`Morton ordering`: http://en.wikipedia.org/wiki/Z-order_curve

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        sorted.las
      </Option>
      <Filter type="filters.mortonorder">
        <Reader type="readers.las">
          <Option name="filename">
            unsorted.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>



Notes
-----

