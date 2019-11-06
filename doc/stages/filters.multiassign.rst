.. _filters.multiassign:

filters.multiassign
===================

The multiassign filter allows you set the values to dimensions for all points
to a provided value that pass a range filter. This is a upgraded version of :ref:`filters.assign`
to allow passing array of assignments with optional condition to every assignment.

.. embed::

.. streamable::

Example 1
---------

This pipeline resets the ``Colors`` of all points to white where classification is >=2, red where classification is 3,
green where classification is 4 and blue where classification is >=5.

.. code-block:: json

  [
      "autzen-dd.las",
      {
        "type":"filters.multiassign",
        "args":{
			"assignments":[
				{
					"assign":"Red[:]=255,Green[:]=255,Blue[:]=255",
					"condition":"Classification[:2]"
				},
				{
					"assign":"Red[:]=255,Green[:]=0,Blue[:]=0",
					"condition":"Classification[3:3]"
				},
				{
					"assign":"Red[:]=0,Green[:]=255,Blue[:]=0",
					"condition":"Classification[4:4]"
				},
				{
					"assign":"Red[:]=0,Green[:]=0,Blue[:]=255",
					"condition":"Classification[5:]"
				}
			]
		}
      },
      {
          "filename":"attributed.las",
          "scale_x":0.0000001,
          "scale_y":0.0000001
      }
  ]

Options
-------

args
  Contains arguments for assignment.
  
  assignment
	A comma separated list of assignmet and condition pairs. Each pair for assignment is executed sequentially.

	assign
		A comma separated list of :ref:`range <ranges>` followed by an assignment of a value (see example).
	
	condition
		A :ref:`ranges <ranges>` that a point's values must pass in order for the assignment to be performed. [Default: none]
