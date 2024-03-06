.. _filters.h3:

filters.h3
===========================

The **H3 filter** adds a `H3 <https://h3geo.org/docs/api/indexing/>`__ ID at a given ``resolution``. The
`uint64_t` integer corresponds to the H3 index of the point.


.. streamable::

.. warning::

    :ref:`filters.h3` internally depends on being able to reproject the coordinate system to ``EPSG:4326``.
    If the data does not have coordinate system information, the filter will throw an error.

Options
-------

resolution
  The H3 resolution [Default: 0]

