.. _filters.mortonorder:

filters.mortonorder
================================================================================

Sorts the XY data using `Morton ordering`_.

.. _`Morton ordering`: http://en.wikipedia.org/wiki/Z-order_curve

.. embed::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "uncompressed.las",
        {
          "type":"filters.mortonorder"
        },
        {
          "type":"writers.las",
          "filename":"compressed.laz",
          "compression":"true"
        }
      ]
    }


Notes
-----

