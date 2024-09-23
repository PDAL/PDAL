.. _filters.randomize:

filters.randomize
=================

The randomize filter reorders the points in a point view randomly.

.. embed::

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.randomize"
      },
      {
          "type":"writers.las",
          "filename":"output.las"
      }
  ]

Options
-------

.. include:: filter_opts.rst

