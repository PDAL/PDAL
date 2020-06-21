.. _ground_command:

********************************************************************************
ground
********************************************************************************

The ``ground`` command is used to segment the input point cloud into ground
versus non-ground returns using :ref:`filters.smrf` and :ref:`filters.outlier`.

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
  --extract           Extract ground returns?
  --reset             Reset classifications prior to segmenting?
  --denoise           Apply statistical outlier removal prior to segmenting?
  --returns           Include last returns?
  --scalar            Elevation scalar?
  --threshold         Elevation threshold?
  --cut               Cut net size?
  --ignore            A range query to ignore when processing


