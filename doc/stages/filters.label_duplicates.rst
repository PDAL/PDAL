.. _filters.label_duplicates:

filters.label_duplicates
==========================

:ref:`filters.label_duplicates` assigns a ``Duplicate`` :ref:`dimensions` value
to ``1`` if all of the dimensions listed in the ``dimensions`` option
for the points are equal.

.. embed::

.. warning::

    The filter **requires** the data to be sorted **before** the labeling can
    work. It simply checks the dimensions and points in order, and if each
    dimension is equal from one point to the next, it is labeled a duplicate.

Example
-------


.. code-block:: json

  [
      "unsorted.las",
      {
          "type":"filters.sort",
          "dimension":"X"
      },
      {
          "type":"filters.sort",
          "dimension":"Y"
      },
      {
          "type":"filters.sort",
          "dimension":"Z"
      },
      {
          "type":"filters.sort",
          "dimension":"GPStime"
      },
      {
          "type":"filters.label_duplicates",
          "dimensions":"X,Y,Z,GPStime"
      },
      "duplicates.txt"
  ]


Options
-------

_`dimensions`
  The dimensions which must be equal for the point to be declared a duplicate. [Required]


.. include:: filter_opts.rst

