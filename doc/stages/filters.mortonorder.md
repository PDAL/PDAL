.. _filters.mortonorder:

filters.mortonorder
================================================================================

Sorts the XY data using `Morton ordering`_.

It's also possible to compute a reverse Morton code by reading the binary
representation from the end to the beginning. This way, points are sorted
with a good dispersement. For example, by successively selecting N
representative points within tiles:

.. figure:: filters.mortonorder.img1.png
    :scale: 100 %
    :alt: Reverse Morton indexing

.. _`Morton ordering`: http://en.wikipedia.org/wiki/Z-order_curve

.. seealso::

    See `LOPoCS`_ and `pgmorton`_ for some use case examples of the
    Reverse Morton algorithm.

.. _`pgmorton`: https://github.com/Oslandia/pgmorton
.. _`LOPoCS`: https://github.com/Oslandia/lopocs

.. embed::

Example
-------

.. code-block:: json

  [
      "uncompressed.las",
      {
          "type":"filters.mortonorder",
          "reverse":"false"
      },
      {
          "type":"writers.las",
          "filename":"compressed.laz",
          "compression":"true"
      }
  ]


Options
--------

.. include:: filter_opts.rst

