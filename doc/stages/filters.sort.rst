.. _filters.sort:

filters.sort
============

The sort filter orders a point view based on the values of a dimension_. The
sorting can be done in increasing (ascending) or decreasing (descending) order_.

.. embed::

Example
-------


.. code-block:: json

  [
      "unsorted.las",
      {
          "type":"filters.sort",
          "dimension":"X",
          "order":"ASC"
      },
      "sorted.las"
  ]


Options
-------

_`dimension`
  The dimension on which to sort the points. [Required]

_`order`
  The order in which to sort, ASC or DESC [Default: "ASC"]
