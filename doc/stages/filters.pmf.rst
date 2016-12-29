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

Notes
-------------------------------------------------------------------------------

* ``slope`` controls the height threshold at each iteration. A slope of ``1.0``
  represents a 1:1 or 45º.

* ``initial_distance`` is _intended_ to be set to account for z noise, so for a
  flat surface if you have an uncertainty of around 15 cm, you set
  ``initial_distance`` large enough to not exclude these points from the ground.

* For a given iteration, the height threshold is determined by multiplying
  ``slope`` by ``cell_size`` by the difference in window size between the current
  and last iteration, plus the ``initial_distance``. This height threshold is
  constant across all cells and is maxed out at the ``max_distance`` value. If
  the difference in elevation between a point and its “opened” value (from the
  morphological operator) exceeds the height threshold, it is treated as
  non-ground.  So, bigger slope leads to bigger height thresholds, and these
  grow with each iteration (not to exceed the max).  With flat terrain,
  keep this low, the thresholds are small, and stuff is more aggressively
  dumped into non-ground class.  In rugged terrain, open things up
  a little, but then you can start missing buildings, veg, etc.

* Very large ``max_window_size`` values will result in a lot of potentially
  extra iteration. This parameter can have a strongly negative impact on
  computation performance.

.. note::
    [Zhang2003]_ describes the consequences and relationships of the
    parameters in more detail and is the canonnical resource on the
    topic.

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
