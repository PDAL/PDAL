.. _filters.pmf:

filters.pmf
===============================================================================

The Progressive Morphological Filter (PMF) is a method of segmenting ground and
non-ground returns. This filter is an implementation of the method described in
[Zhang2003]_.


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

max_window_size
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
