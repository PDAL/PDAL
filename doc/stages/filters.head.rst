.. _filters.head:

filters.head
===============================================================================

The **Head filter** returns a specified number of points from the beginning
of a ``PointView``.

.. note::

    If the requested number of points exceeds the size of the point cloud, all
    points are passed with a warning.

.. embed::


Example #1
----------

Thin a point cloud by first shuffling the point order with
:ref:`filters.randomize` and then picking the first 10000 using the HeadFilter.


.. code-block:: json

  [
      {
          "type":"filters.randomize"
      },
      {
          "type":"filters.head",
          "count":10000
      }
  ]


Example #2
----------

Compute height above ground and extract the ten highest points.


.. code-block:: json

  [
      {
          "type":"filters.smrf"
      },
      {
          "type":"filters.hag"
      },
      {
          "type":"filters.sort",
          "dimension":"HeightAboveGround",
          "order":"DESC"
      },
      {
          "type":"filters.head",
          "count":10
      }
  ]

.. seealso::

    :ref:`filters.tail` is the dual to :ref:`filters.head`.


Options
-------------------------------------------------------------------------------

count
  Number of points to return. [Default: 10]
