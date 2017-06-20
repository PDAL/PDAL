.. _filters.tail:

filters.tail
===============================================================================

The TailFilter returns a specified number of points from the end of the
PointView.

.. note::

    If the requested number of points exceeds the size of the point cloud, all
    points are passed with a warning.


Example #1
----------

Sort and extract the 100 lowest intensity points.


.. code-block:: json

    {
      "pipeline":[
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
    }


.. seealso::

    :ref:`filters.head` is the dual to :ref:`filters.tail`.


Options
-------------------------------------------------------------------------------

count
  Number of points to return. [Default: **10**]
