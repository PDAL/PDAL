.. _filters.locate:

filters.locate
===============================================================================

The Locate filter searches the specified dimension_ for the minimum or
maximum value and returns a single point at this location. If multiple points
share the min/max value, the first will be returned. All dimensions of the
input ``PointView`` will be output, subject to any overriding writer options.

.. embed::

Example
-------

This example returns the point at the highest elevation.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.locate",
          "dimension":"Z",
          "minmax":"max"
      },
      "output.las"
  ]

Options
-------

_`dimension`
  Name of the dimension in which to search for min/max value.

minmax
  Whether to return the minimum or maximum value in the dimension.
