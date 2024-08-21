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
    The `STABLE` algorithm **must** be set or it will fail to properly label
    duplicates.

Example
-------


.. code-block:: json

  [
      "unsorted.las",
      {
          "type":"filters.sort",
          "algorithm":"STABLE",
          "dimension":"X"
      },
      {
          "type":"filters.sort",
          "algorithm":"STABLE",
          "dimension":"Y"
      },
      {
          "type":"filters.sort",
          "algorithm":"STABLE",
          "dimension":"Z"
      },
      {
          "type":"filters.sort",
          "algorithm":"STABLE",
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

