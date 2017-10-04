.. _density_command:

********************************************************************************
density
********************************************************************************

The density command produces a tessellated hexagonal `OGR layer`_ from the
output of :ref:`filters.hexbin`.

.. _`OGR layer`: http://www.gdal.org/ogr_utilities.html

.. note::

    The ``density`` command is only available when PDAL is linked with Hexer
    (BUILD_PLUGIN_HEXBIN=ON).

::

    $ pdal density <input> <output>

::

    --input, -i        Input point cloud file name
    --output, -o       Output vector data source
    --lyr_name         OGR layer name to write into datasource
    --ogrdriver, -f    OGR driver name to use
    --sample_size      Sample size for automatic edge length calculation. [5000]
    --threshold        Required cell density [15]
    --hole_cull_tolerance_area
                       Tolerance area to apply to holes before cull
    --smooth           Smooth boundary output
