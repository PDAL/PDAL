.. _ground_command:

********************************************************************************
ground
********************************************************************************

The ``ground`` command is used to segment the input point cloud into ground
versus non-ground returns. The output is a point cloud containing only ground
returns. The ``ground`` command invokes `Point Cloud Library
<http://pointclouds.org/>`_'s `ProgressiveMorphologicalFilter`_.

.. note::

    The ``ground`` command is only available when PDAL is linked with PCL.

::

    $ pdal ground <input> <output>

::

    --input [-i] arg       Non-positional option for specifying input filename
    --output [-o] arg      Non-positional option for specifying output filename
    --maxWindowSize arg    max window size [33]
    --slope arg            slope [1]
    --maxDistance arg      max distance [2.5]
    --initialDistance arg  initial distance [0.15]
    --cellSize arg         cell size [1]
    --classify             apply classification labels? [true]
    --extract              extract ground returns? [false]
    --approximate [-a]     Use significantly faster approximate algorithm? [false]

.. _`ProgressiveMorphologicalFilter`: http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php#progressive-morphological-filtering.
