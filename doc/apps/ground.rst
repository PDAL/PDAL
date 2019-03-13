.. _ground_command:

********************************************************************************
ground
********************************************************************************

The ``ground`` command is used to segment the input point cloud into ground
versus non-ground returns.

::

    $ pdal ground [options] <input> <output>

::

    --input, -i         Input filename
    --output, -o        Output filename
    --max_window_size   Max window size
    --slope             Slope
    --max_distance      Max distance
    --initial_distance  Initial distance
    --cell_size         Cell size
    --classify          Apply classification labels?
    --extract           Extract ground returns?
    --approximate, -a   Use approximate algorithm? (much faster)
    --reset             Reset classification prior to segmentation.
    --denoise           Apply statistical outlier removal prior to segmentation.

  For more information, see the full documentation for PDAL at http://pdal.io/
