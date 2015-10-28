.. _filters.randomize:

filters.randomize
=================

The randomize filter reorders the points in a point view randomly.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        randomized.las
      </Option>
      <Filter type="filters.randomize">
        <Reader type="readers.las">
          <Option name="filename">
            unsorted.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

