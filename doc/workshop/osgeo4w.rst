.. _workshop-osgeo4w:


OSGeo4W
================================================================================

.. index:: OSGeo4W, software installation

.. include:: ./includes/substitutions.rst


What is OSGeo4W?
--------------------------------------------------------------------------------

|OSGeo4W| is a distribution of geospatial software built for Windows. The PDAL
project provides Windows builds through this distribution.


How will we use OSGeo4W?
--------------------------------------------------------------------------------

PDAL stands on the shoulders of giants. It uses GDAL, GEOS, and
:ref:`many other dependencies <building>`. Because of this, it is very
challenging to build it yourself. We could easily burn an entire workshop
learning the esoteric build miseries of PDAL and all of its dependencies.
Fortunately, OSGeo4W provides us a fully-featured known
configuration to run our examples and exercises without having to suffer
so much.


Installing OSGeo4W
--------------------------------------------------------------------------------

1. Copy the entire contents of your workshop USB key to a ``PDAL`` directory in your
   home directory (something like ``C:\Users\hobu\PDAL``). We will refer to
   this location for the rest of the workshop materials.

2. Download the OSGeo4W setup.exe http://download.osgeo.org/osgeo4w/osgeo4w-setup-x86_64.exe

   .. warning::

       PDAL is only available via the 64-bit verison of OSGeo4W. There is an older
       32-bit version, but none of the workshop materials will work in that
       environment. You must be able to run 64-bit Windows applications to follow
       this workshop.

3. Run the installer and choose the "Advanced Install" option.

   .. image:: ./images/osgeo4w-advanced-selection.png

4. Search for "PDAL" in the search box or drill down through the Commandline_Utilities
   section and choose the ``PDAL`` package.

   .. image:: ./images/osgeo4w-pdal-selection.png

5. Search for "QGIS" in the search box or drill down through the Desktop
   section and choose the ``QGIS`` package. This will be used in a later
   section of the workshop.

   .. image:: ./images/osgeo4w-qgis-selection.png

6. You will probably get a screen telling you that you have unmet dependencies.
   You must check the box next to ``Install these packages to meet dependencies (RECOMMENDED)``
   for PDAL to work.

   .. image:: ./images/osgeo4w-dep-selection.png

7. You will probably get at least one screen asking you to accept a license agreement.
   You must check the box next to ``I agreed with the above license terms`` on each screen
   for PDAL to work.

   .. image:: ./images/osgeo4w-license-selection.png
