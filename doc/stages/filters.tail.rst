.. _filters.tail:

filters.tail
===============================================================================

The **Tail Filter** returns a specified number of points from the end of the
``PointView``.

.. note::

    If the requested number of points exceeds the size of the point cloud, all
    points are passed with a warning.

.. embed::

Example
-------

Sort and extract the 100 lowest intensity points.


.. code-block:: json

  [
      {
          "type":"filters.sort",
          "dimension":"Intensity",
          "order":"DESC"
      },
      {
          "type":"filters.tail",
          "count":100
      }
  ]


.. seealso::

    :ref:`filters.head` is the dual to :ref:`filters.tail`.


Options
-------------------------------------------------------------------------------

count
  Number of points to return. [Default: 10]
