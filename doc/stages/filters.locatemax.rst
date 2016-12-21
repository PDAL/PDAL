.. _filters.locatemax:

===============================================================================
filters.locatemax
===============================================================================

The Locate Max filter searches the specified ``dimension`` for the maximum value
and returns a single point at this location. If multiple points share the max
value, the first will be returned. All dimensions of the input ``PointView``
will be output, subject to any overriding writer options.

Example
-------

This example returns the point at the highest elevation.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.locatemax",
          "dimension":"Z"
        },
        "output.las"
      ]
    }

Options
-------

dimension
  Name of the dimension in which to search for max value.
