.. _filters.stats:

filters.stats
===============================================================================

The stats filter calculates the minimum, maximum and average (mean) values
of dimensions.  On request it will also provide an enumeration of values of
a dimension.

The output of the stats filter is metadata that can be stored by writers or
used through the PDAL API.  Output from the stats filter can also be
quickly obtained in JSON format by using the command ``pdal info --stats``.


Example
................................................................................

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.stats",
          "dimensions":"X,Y,Z,Classification",
          "enumerate":"Classification"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }


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

global
  A comma-separated list of dimensions for which global statistics (median,
  mad, mode) should be calculated.

advanced
  Calculate advanced statistics (skewness, kurtosis). [Default: false]

