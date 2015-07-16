.. _filters.stats:

filters.stats
============

The stats filter calculates the minimum, maximum and average (mean) values
of dimensions.  On request it will also provide an enumeration of values of
a dimension.

The output of the stats filter is metadata that can be stored by writers or
used through the PDAL API.  Output from the stats filter can also be
quickly obtained in JSON format by using the command ``pdal info --stats``.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        sorted.las
      </Option>
      <Filter type="filters.stats">
        <Option name="dimensions">X,Y,Z,Classification</Option>
        <Option name="enumerate">Classification</Option>
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

dimensions
  A comma-separated list of dimensions whose statistics should be
  processed.  If not provided, statistics for all dimensions are calculated.
  
enumerate
  A comma-separated list of dimensions whose values should be enumerated.
  Note that this list does not add to the list of dimensions that may be
  provided in the **dimensions** option.

count
  Identical to the --enumerate option, but provides a count of the number
  of points in each enumerated category.
