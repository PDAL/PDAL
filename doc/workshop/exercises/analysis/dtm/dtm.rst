.. _dtm:

Generating a DTM
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: elevation model, DTM, DSM

This exercise uses PDAL to generate an elevation model surface using the
output from the :ref:`ground` exercise, PDAL's :ref:`writers.gdal` operation,
and |GDAL| to generate an elevation and hillshade surface from point cloud
data.


Exercise
--------------------------------------------------------------------------------


.. note::

    The primary input for `Digital Terrain Model`_ generation is a point cloud
    with ground classifications. We created this file, called
    ``ground-filtered.laz``, in the :ref:`ground` exercise. Please produce that
    file by following that exercise before starting this one.

.. _`Digital Terrain Model`: https://en.wikipedia.org/wiki/Digital_elevation_model





Command
................................................................................

Invoke the following command, substituting accordingly, in your `Docker
Quickstart Terminal`:

PDAL capability to generate rasterized output is provided by the :ref:`writers.gdal`
stage. There is no :ref:`application <apps>` to drive this stage, and we
must use a pipeline.

Pipeline breakdown
................................................................................


.. include:: ./gdal.json
    :literal:

.. note::

    this pipeline is available in your workshop materials in the
    ``./exercises/analysis/dtm/dtm.json`` file.


1. Reader
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``ground-filtered.laz`` is the |LASzip| file we will clip. You should have
created this output as part of the :ref:`ground` exercise.


2. :ref:`writers.gdal`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `Points2grid`_ writer that bins the point cloud data into an elevation
surface.


.. _`Points2grid`: https://github.com/CRREL/points2grid



Execution
................................................................................

.. literalinclude:: ./dtm-run-command.txt
    :linenos:

.. image:: ../../../images/dtm-run-command.png

Visualization
................................................................................

Something happened, and some files were written, but we cannot really
see what was produced. Let us use :ref:`qgis` to visualize the output.

1. Open :ref:`qgis` and `Add Raster Layer`:

   .. image:: ../../../images/dtm-add-raster-layer.png

2. Add the `dtm.idw.tif` file from your ``./PDAL/exercises/analysis/dtm``
   directory.

   .. image:: ../../../images/dtm-add-raster-mean.png

   .. image:: ../../../images/dtm-qgis-added.png

3. Classify the DTM by right-clicking on the `dtm.idw.tif` and choosing
   `Properties`. Pick the pseudocolor rendering type, and then
   choose a color ramp and click `Classify`.

   .. image:: ../../../images/dtm-qgis-classify.png

   .. image:: ../../../images/dtm-qgis-colorize-dtm.png


4. :ref:`qgis` provides access to |GDAL| processing tools, and we
   are going to use that to create a hillshade of our surface.
   Choose `Raster-->Analysis-->Dem`:

   .. image:: ../../../images/dtm-qgis-select-hillshade.png

5. Click the window for the `Output file` and select a location
   to save the ``hillshade.tif`` file.

   .. image:: ../../../images/dtm-qgis-gdaldem.png

6. Click `OK` and the hillshade of your DTM is now available

   .. image:: ../../../images/dtm-qgis-hillshade-done.png

Notes
--------------------------------------------------------------------------------

1. `gdaldem`_, which powers the :ref:`qgis` DEM tools, is a very powerful
   command line utility you can use for processing data.

2. `Points2grid`_ can be used for large data, but it does not interplate
   typical `TIN`_ surface model before interpolating.

.. _`TIN`: https://en.wikipedia.org/wiki/Triangulated_irregular_network
.. _`gdaldem`: http://www.gdal.org/gdaldem.html
