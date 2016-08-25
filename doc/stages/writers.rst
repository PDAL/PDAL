.. _writers:

Writers
=======

Writers consume data provided by :ref:`readers`. Some writers can consume any
dimension type, while others only understand fixed dimension names.

.. note::

    Some writers can only consume known dimension names, while PDAL doesn't
    yet have a registery for the dimension types, you can see the
    base dimension types at https://github.com/PDAL/PDAL/blob/master/include/pdal/Dimension.hpp

.. toctree::
   :maxdepth: 1
   :glob:

   writers.*

