.. _writers.ept_addon:

writers.ept_addon
=================

The **EPT Addon Writer** supports writing additional dimensions to
`Entwine Point Tile`_ datasets.  The EPT addon writer may only
be used in a pipeline with an :ref:`EPT reader <readers.ept>`, and it
creates additional attributes for an existing dataset rather than
creating a brand new one.

The addon dimensions created by this writer are stored independently from the corresponding EPT dataset, therefore write-access to the EPT resource itself is not required to create and use addon dimensions.

.. embed::

Example
--------------------------------------------------------------------------------

This example downloads the Autzen dataset (10M points) and runs the
:ref:`SMRF filter <filters.smrf>`, which populates the ``Classification``
dimension with ground values, and writes the resulting attribute to an EPT
addon dataset on the local filesystem.

.. code-block:: json

  [
      {
          "type": "readers.ept",
           "filename": "http://na.entwine.io/autzen/ept.json"
      },
      {
          "type": "filters.assign",
          "assignment": "Classification[:]=0"
      },
      {
          "type": "filters.smrf"
      },
      {
          "type": "writers.ept_addon",
          "addons": { "~/entwine/addons/autzen/smrf": "Classification" }
      }
  ]

And here is a follow-up example of reading this dataset with the
:ref:`EPT reader <readers.ept>` with the created addon overwriting the
``Classification`` value.  The output is then written to a single file
with the :ref:`LAS writer <writers.las>`.

.. code-block:: json

  [
      {
          "type": "readers.ept",
          "filename": "http://na.entwine.io/autzen/ept.json",
          "addons": { "Classification": "~/entwine/addons/autzen/smrf" }
      },
      {
          "type": "writers.las",
          "filename": "autzen-ept-smrf.las"
      }
  ]

This is an example of using multiple mappings in the ``addons`` option to
apply a new color scheme with :ref:`filters.colorinterp` mapping the
Red, Green, and Blue dimensions to new values.

.. code-block:: json

  [
      {
          "type": "readers.ept",
          "filename": "http://na.entwine.io/autzen/ept.json"
      },
      {
          "type": "filters.colorinterp"
      },
      {
          "type": "writers.ept_addon",
          "addons": {
              "~/entwine/addons/autzen/interp/Red":   "Red",
              "~/entwine/addons/autzen/interp/Green": "Green",
              "~/entwine/addons/autzen/interp/Blue":  "Blue"
          }
      }
  ]

The following pipeline will read the data with the new colors:

.. code-block:: json

  [
      {
          "type": "readers.ept",
          "filename": "http://na.entwine.io/autzen/ept.json",
          "addons": {
              "Red":   "~/entwine/addons/autzen/interp/Red",
              "Green": "~/entwine/addons/autzen/interp/Green",
              "Blue":  "~/entwine/addons/autzen/interp/Blue"
          }
      },
      {
          "type": "writers.las",
          "filename": "autzen-ept-interp.las"
      }
  ]

Options
--------------------------------------------------------------------------------

addons
   A JSON object whose keys represent output paths for each addon dimension,
   and whose corresponding values represent the attributes to be written to
   these addon dimensions. [Required]

.. note::

   The `addons` option is reversed between the EPT reader and addon-writer: in each case, the right-hand side represents an assignment to the left-hand side.  In the writer, the dimension value is assigned to an addon path.  In the reader, the addon path is assigned to a dimension.

threads
    Number of worker threads used to write EPT addon data.  A minimum of 4 will be used no matter what value is specified.

.. _Entwine Point Tile: https://entwine.io/entwine-point-tile.html

