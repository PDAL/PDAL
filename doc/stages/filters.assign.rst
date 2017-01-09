.. _filters.assign:

filters.assign
===================

The assign filter allows you set the value of a dimension for all points
to a provided value.


Example 1
---------

This pipeline sets the PointSourceId of all points from 'autzen-dd.las'
to the value '26'.

.. code-block:: json

    {
      "pipeline":[
        "autzen-dd.las",
        {
          "type":"filters.assign",
          "dimension":"PointSourceId",
          "value":26
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

dimension
  Name of the dimension whose value should be altered.  [Required]

value
  Value to apply to the dimension.  [Required]

