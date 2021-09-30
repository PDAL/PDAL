.. _readers.lruept:

readers.lruept
===========

The **LRUEPT reader** works the same as :ref:`writers.ept` but comes up with a in memory caching mechanism

.. plugin::

.. streamable::

Installation
------------

To build PDAL with lruept support, you need to install Intel TBB.


Example
-------

.. code-block:: json

   [
      {
         "type": "readers.lruept",
         "filename": "http://na.entwine.io/nyc/ept.json",
         "bounds": "([-8242669, -8242529], [4966549, 4966674])"
      },
      "statue-of-liberty.las"
   ]


Options
-------

See :ref:`writers.ept`

