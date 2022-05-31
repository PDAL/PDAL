.. _workshop-dtm:

Generating a DTM
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: elevation model, DTM, DSM

This exercise uses PDAL to generate an elevation model surface using the
output from the :ref:`workshop-ground` exercise, PDAL's :ref:`writers.gdal`
operation, and |GDAL| to generate an elevation and hillshade surface from
point cloud data.


Exercise
--------------------------------------------------------------------------------

.. note::

    The primary input for `Digital Terrain Model`_ generation is a point cloud
    with ground classifications. We created this file, called
    ``denoised-ground-only.laz``, in the :ref:`workshop-ground` exercise. Please produce that
    file by following that exercise before starting this one.

.. _`Digital Terrain Model`: https://en.wikipedia.org/wiki/Digital_elevation_model

Command
................................................................................

Invoke the following command, substituting accordingly, in your |Terminal|:

PDAL capability to generate rasterized output is provided by the
:ref:`writers.gdal` stage. There is no :ref:`application <apps>` to drive this
stage, and we must use a pipeline.

Pipeline breakdown
................................................................................


.. include:: ./gdal.json
    :literal:

.. note::

    This pipeline is available in your workshop materials in the
    ``./exercises/analysis/dtm/gdal.json`` file. Make sure to edit the
    filenames to match your paths.


1. Reader
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``denoised-ground-only`` is the |LASzip| file we will clip. You should have
created this output as part of the :ref:`workshop-ground` exercise.


2. :ref:`writers.gdal`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`writers.gdal` writer that bins the point cloud data into an elevation
surface.

Execution
................................................................................

.. code-block:: console

    $ pdal pipeline ./exercises/analysis/dtm/gdal.json

Visualization
................................................................................

Something happened, and some files were written, but we cannot really
see what was produced. Let us use :ref:`qgis` to visualize the output.

1. Open :ref:`qgis` and `Add Raster Layer`:

    .. image:: ../../../images/dtm-add-raster-layer.png
        :target: ../../../../_images/dtm-add-raster-layer.png

2. Add the `dtm.tif` file from your ``./exercises/analysis/dtm``
   directory.

    .. image:: ../../../images/dtm-add-raster-mean.png
        :target: ../../../../_images/dtm-add-raster-mean.png

    .. image:: ../../../images/dtm-qgis-added-no-fill.png
        :target: ../../../../_images/dtm-qgis-added-no-fill.png

3. Go to Raster -> Analyze -> Fill nodata... and select the default values

    .. image:: ../../../images/dtm-qgis-fill-nodata.png
        :target: ../../../_images/dtm-qgis-fill-nodata.png

    .. image:: ../../../images/dtm-qgis-added.png
        :target: ../../../../_images/dtm-qgis-added.png

4. Classify the DTM by right-clicking on the `Filled` and choosing
   `Properties`. Pick the pseudocolor rendering type, and then
   choose a color ramp and click `Classify`.

    .. image:: ../../../images/dtm-qgis-classify.png
        :target: ../../../../_images/dtm-qgis-classify.png

    .. image:: ../../../images/dtm-qgis-colorize-dtm.png
        :target: ../../../../_images/dtm-qgis-colorize-dtm.png


5. :ref:`qgis` provides access to |GDAL| processing tools, and we
   are going to use that to create a hillshade of our surface.
   Choose `Raster-->Analysis-->Hillshade`:

    .. image:: ../../../images/dtm-qgis-select-hillshade.png
        :target: ../../../../_images/dtm-qgis-select-hillshade.png

6. Click the window for the `Output file` and select a location
   to save the ``hillshade.tif`` file.

    .. image:: ../../../images/dtm-qgis-gdaldem.png
        :target: ../../../../_images/dtm-qgis-gdaldem.png


    .. code-block:: console

        $ gdaldem hillshade ./exercises/analysis/dtm/dtm.tif \
        ./exercises/analysis/dtm/hillshade.tif \
        -z 1.0 -s 1.0 -az 315.0 -alt 45.0 \
        -of GTiff

    .. code-block:: doscon

        > gdaldem hillshade ./exercises/analysis/dtm/dtm.tif ^
        ./exercises/analysis/dtm/hillshade.tif ^
        -z 1.0 -s 1.0 -az 315.0 -alt 45.0 ^
        -of GTiff

7. Click `OK` and the hillshade of your DTM is now available

    .. image:: ../../../images/dtm-qgis-hillshade-done.png
        :target: ../../../../_images/dtm-qgis-hillshade-done.png

Notes
--------------------------------------------------------------------------------

1. `gdaldem`_, which powers the :ref:`qgis` DEM tools, is a very powerful
   command line utility you can use for processing data.

2. :ref:`writers.gdal` can be used for large data, but it does not interpolate
   a typical `TIN`_ surface model.

.. _`TIN`: https://en.wikipedia.org/wiki/Triangulated_irregular_network
.. _`gdaldem`: http://www.gdal.org/gdaldem.html
