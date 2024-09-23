.. _filters.elm:

filters.elm
===============================================================================

The Extended Local Minimum (ELM) filter marks low points as noise. This filter
is an implementation of the method described in [Chen2012]_.

ELM begins by rasterizing the input point cloud data at the given cell_ size.
Within each cell, the lowest point is considered noise if the next lowest point
is a given threshold above the current point. If it is marked as noise, the
difference between the next two points is also considered, marking points as
noise if needed, and continuing until another neighbor is found to be within the
threshold. At this point, iteration for the current cell stops, and the next
cell is considered.

.. embed::

Example #1
----------

The following PDAL pipeline applies the ELM filter, using a cell_ size of 20
and
applying the :ref:`classification <class>` code of 18 to those points
determined to be noise.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.elm",
          "cell":20.0,
          "class":18
        },
        "output.las"
      ]
    }

Example #2
----------

This variation of the pipeline begins by assigning a value of 0 to all
classifications, thus resetting any existing classifications. It then proceeds
to compute ELM with a threshold_ value of 2.0, and finishes by extracting all
returns that are not marked as noise.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.assign",
          "assignment":"Classification[:]=0"
      },
      {
          "type":"filters.elm",
          "threshold":2.0
      },
      {
          "type":"filters.range",
          "limits":"Classification![7:7]"
      },
      "output.las"
  ]

Options
-------------------------------------------------------------------------------

_`cell`
  Cell size. [Default: 10.0]

_`class`
  Classification value to apply to noise points. [Default: 7]

_`threshold`
  Threshold value to identify low noise points. [Default: 1.0]

.. include:: filter_opts.rst

