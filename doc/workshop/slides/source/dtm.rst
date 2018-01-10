.. _dtm:

Generating a DTM
================================================================================


.. include:: ../../includes/substitutions.rst



Purpose:
    * Generate a DTM from ground-filtered data
    * Data was created in :ref:`ground`
    * Visualize DTM using :ref:`qgis`


:ref:`DTM Workshop Materials <pdal:dtm>`


writers.gdal
================================================================================

* :ref:`writers.gdal`
* Ported points2grid (OpenTopography)
* IDW available for fill-in
* Write TIFF/ASCII raster
* Control pixel size

DTM (pipeline)
================================================================================

.. literalinclude:: ../../exercises/analysis/dtm/gdal.json

DTM (execution)
================================================================================

.. literalinclude:: ../../exercises/analysis/dtm/dtm-run-command.txt
    :linenos:

DTM (command)
================================================================================

.. image:: ../../images/dtm-run-command.png

DTM (visualization)
================================================================================

.. image:: ../../images/dtm-add-raster-mean.png

DTM (visualization)
================================================================================

.. image:: ../../images/dtm-qgis-classify.png

DTM (visualization)
================================================================================

.. image:: ../../images/dtm-qgis-colorize-dtm.png

DTM (visualization)
================================================================================

.. image:: ../../images/dtm-qgis-select-hillshade.png

DTM (visualization)
================================================================================

.. image:: ../../images/dtm-qgis-gdaldem.png

DTM (visualization)
================================================================================

.. image:: ../../images/dtm-qgis-hillshade-done.png

Home
================================================================================
:ref:`Home <home>`

