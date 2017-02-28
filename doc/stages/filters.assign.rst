.. _filters.assign:

filters.assign
===================

The assign filter allows you set the value of a dimension for all points
to a provided value that pass a range filter.


Example 1
---------

This pipeline resets the Classification of all points with classiciations
2 or 3 to 0 and all points with classification of 5 to 4.

.. code-block:: json

    {
      "pipeline":[
        "autzen-dd.las",
        {
          "type":"filters.assign",
          "assignment" : "Classification[2:3]=0",
          "assignment" : "Classification[5:5]=4"
        },
        {
          "filename":"attributed.las",
          "scale_x":0.0000001,
          "scale_y":0.0000001
        }
      ]
    }


Options
-------

assignment
  A :ref:`range <ranges>` followed by an assignment of a value (see example).
  Can be specified multiple times.  The assignments are applied sequentially
  to the dimension value as set when the filter began processing.
