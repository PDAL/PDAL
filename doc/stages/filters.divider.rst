.. _filters.divider:

filters.divider
===============================================================================

The divider filter breaks a point view into a set of smaller point views
based on simple criteria.  The number of subsets can be specified explicitly,
or one can specify a maximum point count for each subset.  Additionally,
points can be placed into each subset sequentially (as they appear in the
input) or in round-robin fashion.

Normally points are divided into subsets to facilitate output by writers
that support creating multiple output files with a template (LAS and BPF
are notable examples).

Example
-------

This pipeline will create 10 output files from the input file readers.las.

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">out_#.las</Option>
      <Filter type="filters.divider">
        <Option name="count">10</Option>
        <Reader type="readers.las">
            <Option name="filename">example.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

Options
-------

mode
  A mode of 'partition' will write sequential points to an output view until
  the view meets its predetermined size. 'round_robin' mode will iterate
  through the output views as it writes sequential points.
  [Default: 'partition']

count
  Number of output views.  [Default: none]

capacity
  Maximum number of points in each output view.  Views will contain
  approximately equal numbers of points.  [Default: none]

.. warning::

    You must specify exactly one of either 'count' or 'capacity'.

