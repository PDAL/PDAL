.. _filters.assign:

filters.assign
===================

The assign filter allows you set the value of a dimension for all points
to a provided value that pass a range filter.

.. embed::

Example 1
---------

This pipeline resets the ``Classification`` of all points with classifications
2 or 3 to 0 and all points with classification of 5 to 4.

.. code-block:: json

  [
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

Example 2
---------

This pipeline sets some dimensions using arithmetic expressions even 
fetching values from other attributes.

.. code-block:: json

    {
      "pipeline":[
        "autzen-dd.las",
        {
          "type":"filters.assign",
          "assignment" : "Classification[1:1]=Classification * (Z > 1000 || (Z >= 10 && Z <= 20))",
          "assignment" : "Red[:]=(Blue + Green)",
          "assignment" : "Userdata[:]=1000 + (Classification * 2)",
          "assignment" : "UserData[:]=PointSourceId IN (200,500,700)",
          "assignment" : "Z[:]=Z + 100.0",
        },
        {
          "filename":"attributed.las",
        }
      ]
    }

It supports the most common operators (``*,/,+,-,==,!=,<>,>,<,>=,<=,&&,||``), and ``( )`` 
for priorizing them. It supports too ``IN`` function to check the presence of a value 
e.g. ``"PointSourceId IN (2300,2301,2302)"``.


Options
-------

assignment
  A :ref:`range <ranges>` followed by an assignment of a value or arithmetic 
  expression (see examples).
  Can be specified multiple times.  The assignments are applied sequentially
  to the dimension value as set when the filter began processing. [Required]

condition
  A list of :ref:`ranges <ranges>` that a point's values must pass in order
  for the assignment to be performed. [Default: none]
