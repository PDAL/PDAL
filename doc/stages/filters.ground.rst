.. _filters.ground:

===============================================================================
filters.ground
===============================================================================

The Ground filter passes data through the Point Cloud Library (`PCL`_)
ProgressiveMorphologicalFilter algorithm.

ProgressiveMorphologicalFilter is an implementation of the method described in
[Zhang2003]_.

.. [Zhang2003] Zhang, Keqi, et al. "A progressive morphological filter for removing nonground measurements from airborne LIDAR data." Geoscience and Remote Sensing, IEEE Transactions on 41.4 (2003): 872-882.

.. _`PCL`: http://www.pointclouds.org

Options
-------------------------------------------------------------------------------

maxWindowSize
  Maximum window size. [Default: **33**]

slope
  Slope. [Default: **1.0**]

maxDistance
  Maximum distance. [Default: **2.5**]

initialDistance
  Initial distance. [Default: **0.15**]

cellSize
  Cell Size. [Default: **1**]

classify
  Apply classification labels? [Default: **true**]

extract
  Extract ground returns? [Default: **false**]

approximate
  Use approximate algorithm? [Default:: **false**]
