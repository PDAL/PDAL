.. _filters.groupby:

filters.groupby
===============================================================================

The groupby filter takes a single PointView as its input and creates an output
PointView for each category in the named ``dimension`` or each range in
``ranges``.

.. embed::

Example #1
----------

The first example pipeline will create a new output PointView for each unique
classification value.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.groupby",
          "dimension":"Classification"
        },
        "output_#.las"
      ]
    }

Example #2
----------

The second example pipeline will create a new output PointView for each
specified range, in this case a ``ReturnNumber`` of 1, or a ``ReturnNumber`` of
anything but 1.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.groupby",
          "ranges":"ReturnNumber[1:1],ReturnNumber![1:1]"
        },
        "output_#.las"
      ]
    }

Options
-------

dimension
  The dimension containing data to be grouped.

ranges
  A comma-separated list of :ref:`ranges` to be grouped.

.. warning::

    You must specify exactly one of either ``dimension`` or ``ranges``.
