.. _dimensions:

===============================================================================
Dimensions
===============================================================================

All point data in PDAL is stored as a set of dimensions.  Dimensions
have a name and a data type.  The data type is determined at runtime, but
a default data type for each dimension is listed below, along with the name
of the dimension and its description.

The following table provides a list of known dimension names you can use in
:ref:`filters`, :ref:`writers`, and :ref:`readers`.

.. note::
    Types are default types. Stage developers should set the dimension type
    explicitly if the default dimension isn't suitable.


.. csv-table::
    :file: ./dimension-table.csv
    :header: "Name", "Type", "Description"
    :widths: 10, 5, 40
