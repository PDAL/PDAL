.. _filters.sort:

filters.sort
============

The sort filter orders a point view based on the values of a dimension. The
sorting can be done in increasing (ascending) or decreasing (descending) order.

.. embed::

Example
-------


.. code-block:: json

    {
      "pipeline":[
        "unsorted.las",
        {
          "type":"filters.sort",
          "dimension":"X",
          "order":"ASC"
        },
        "sorted.las"
      ]
    }


Options
-------

dimension
  The dimension on which to sort the points.

order
  The order in which to sort, ASC or DESC [Default: **ASC**]
