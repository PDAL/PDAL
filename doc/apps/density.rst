.. _density_command:

********************************************************************************
density
********************************************************************************

The density command produces a tessellated hexagonal OGR layer from the
output of :ref:`filters.hexbin`.

.. note::

    The ``density`` command is only available when PDAL is linked with Hexer.

::

    --input, -i        input point cloud file name
    --output, -o       output vector data source
    --lyr_name         OGR layer name to write into datasource
    --ogrdriver, -f    OGR driver name to use
