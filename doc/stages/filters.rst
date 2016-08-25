.. _filters:

Filters
=======

Filters operate on data as inline operations. They can remove, modify, reorganize,
and add points to the data stream as it goes by. Some filters can only operate on
dimensions they understand (consider :ref:`filters.reprojection` doing geographic
reprojection on XYZ coordinates), while others do not interrogate the point data at
all and simply reorganize or split data.

.. toctree::
   :maxdepth: 1
   :glob:

   filters.*
