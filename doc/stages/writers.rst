.. _writers:

Writers
=======

Writers consume data provided by :ref:`readers`. Some writers can consume any
dimension type, while others only understand fixed dimension names.

.. note::

    Some writers can only consume known dimension names. PDAL utilizes a
    ``Dimension.json`` file that describes PDAL dimensions and types.
    You can see it at https://github.com/PDAL/PDAL/blob/master/src/Dimension.json

.. toctree::
   :maxdepth: 1
   :glob:

   writers.*

