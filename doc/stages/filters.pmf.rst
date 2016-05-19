.. _filters.pmf:

filters.pmf
===============================================================================

The Progressive Morphological Filter (PMF) is a method of segmenting ground and
non-ground returns. This filter is an implementation of the method described in
[Zhang2003]_.

.. [Zhang2003] Zhang, Keqi, et al. "A progressive morphological filter for removing nonground measurements from airborne LIDAR data." Geoscience and Remote Sensing, IEEE Transactions on 41.4 (2003): 872-882.


Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.pmf"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }

Options
-------------------------------------------------------------------------------

maxWindowSize
  Maximum window size. [Default: **33**]

slope
  Slope. [Default: **1.0**]

max_distance
  Maximum distance. [Default: **2.5**]

initial_distance
  Initial distance. [Default: **0.15**]

cell_size
  Cell Size. [Default: **1**]

classify
  Apply classification labels? [Default: **true**]

extract
  Extract ground returns? [Default: **false**]

approximate
  Use approximate algorithm? [Default:: **false**]
