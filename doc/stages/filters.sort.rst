.. _filters.sort:

filters.sort
============

The sort filter orders a point view based on the values of a :ref:`dimensions`. The
sorting can be done in increasing (ascending) or decreasing (descending) order.

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

.. note::

    See :ref:`filters.label_duplicates` for an example of using :ref:`filters.sort` to
    sort multiple dimensions at once.

Options
-------

dimensions
  A list of dimensions in the order on which to sort the points. [Required]

order
  The order in which to sort, ASC or DESC [Default: "ASC"]

.. include:: filter_opts.rst

