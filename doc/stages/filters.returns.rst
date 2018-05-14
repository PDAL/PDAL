.. _filters.returns:

filters.returns
===============================================================================

The returns filter takes a single PointView as its input and creates a PointView
for each of the user-specified ``groups`` defined below.

``first`` is defined as those points whose ``ReturnNumber`` is 1 when the ``NumberOfReturns`` is greater than 1.

``intermediate`` is defined as those points whose ``ReturnNumber`` is greater than 1 and less than ``NumberOfReturns`` when ``NumberOfReturns`` is greater than 2.

``last`` is defined as those points whose ``ReturnNumber`` is equal to ``NumberOfReturns`` when ``NumberOfReturns`` is greater than 1.

``only`` is defined as those points whose ``NumberOfReturns`` is 1.

.. embed::

Example
-------

This example creates separate output files for the ``last`` and ``only`` returns.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.returns",
          "groups":"last,only"
        },
        "output_#.las"
      ]
    }

Options
-------

groups
  Comma-separated list of return number groupings ('first', 'last', 'intermediate', or 'only')
