.. _writers.greyhound:

writers.greyhound
=================

The **Greyhound writer** allows you to append new dimensions (or update
existing appended dimensions) onto a Greyhound resource.  The Greyhound writer
may only be used in a pipeline downstream from a Greyhound reader.
Written points must also match the points that were read in a 1:1 manner
(although there is no restriction against reordering), so intermediate
filters that cull points are not allowed.

Example
-------

This example runs the :ref:`SMRF <filters.smrf>` ground-classification
algorithm to populate the **Classification** dimension, the
:ref:`ferry filter <filters.ferry>` to copy
this dimension to a new dimension named **Smrf**, and then adds this
dimension to the Greyhound resource.

.. code-block:: json

  [
      {
          "type": "readers.greyhound",
          "url": "data.greyhound.io/resource/autzen"
      },
      {
          "type": "filters.smrf"
      },
      {
          "type": "filters.ferry",
          "dimensions": "Classification=Smrf"
      },
      {
          "type": "writers.greyhound",
          "name": "MySmrfDimensionSet",
          "dims": ["Smrf"]
      }
  ]

Options
-------

Many options that affect the behavior of the Greyhound writer are specified
as parameters to the :ref:`Greyhound reader <readers.greyhound>`, which are
forwarded to the Greyhound writer.

_`name`
  A string value for the name of this dimension set.  This setting is
  used internally by the Greyhound server to group clusters of
  simultaneously appended dimensions.  For example, a group of
  multiple classifications with different filter settings like
  *["SmrfCellSize_1.0", "SmrfCellSize_1.5"]* might be grouped as
  *"SmrfComparison"*.

_`dims`
  A JSON array of string names of the dimensions to be written to
  the Greyhound resource.

