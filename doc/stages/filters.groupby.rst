.. _filters.groupby:

filters.groupby
===============================================================================

The groupby filter takes a single PointView as its input and creates a PointView
for each category in the named ``dimension`` as its output.

.. embed::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.groupby",
          "dimension":"Classification"
        },
        "output_#.las"
      ]
    }

Options
-------

dimension
  The dimension containing data to be grouped.
