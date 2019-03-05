.. _writers.ept_addon:

writers.ept_addon
=================

The **EPT addon writer** supports writing additional dimensions to `Entwine Point Tile`_ datasets.  Thus the EPT addon writer is unique among most other PDAL writers due to its limitation that it may only be used along with an `EPT reader`_ and modifies and existing dataset rather than creating a brand new one.

The addon dimensions created by this writer may be stored independently from the corresponding EPT dataset, therefore write-access to the EPT resource itself is not required to create and use addon dimensions.

.. embed::

Example
--------------------------------------------------------------------------------

This example downloads the Autzen dataset (10M points) and runs the `SMRF filter`_, which populates the ``Classification`` dimension with ground values, and writes the resulting attribute to an EPT addon dataset on the local filesystem.

.. code-block:: json

   [{
      "type": "readers.ept",
      "filename": "http://na.entwine.io/autzen/ept.json",
      "threads": 12
   }, {
      "type": "filters.assign",
      "assignment": "Classification[:]=0"
   }, {
      "type": "filters.smrf"
   }, {
      "type": "writers.ept_addon",
      "addons": { "~/entwine/addons/autzen/smrf": "Classification" },
      "threads": 12
   }]

And here is a follow-up example of reading this dataset with the `EPT reader`_ - with the created addon overwriting the ``Classification`` value - and then writing the output to a single file with the `LAS writer`_.

.. code-block:: json

   [{
      "type": "readers.ept",
      "filename": "http://na.entwine.io/autzen/ept.json",
      "addons": { "Classification": "~/entwine/addons/autzen/smrf" },
      "threads": 12
   }, {
      "type": "writers.las",
      "filename": "autzen-ept-smrf.las"
   }]


Options
--------------------------------------------------------------------------------

addons
   A JSON object whose keys represent output paths for each addon dimension, and whose corresponding values represent the attributes to be written to these addon dimensions.

.. note::

   The ``addons`` option is reversed between the EPT reader and addon-writer: in each case, the right-hand side represents an assignment to the left-hand side.  In the writer, the dimension value is assigned to an addon path.  In the reader, the addon path is assigned to a dimension.

threads
    Number of worker threads used to write EPT addon data.  A minimum of 4 will be used no matter what value is specified.

.. _Entwine Point Tile: https://github.com/connormanning/entwine/blob/master/doc/entwine-point-tile.md
.. _EPT reader: https://pdal.io/stages/readers.ept.html
.. _SMRF filter: https://pdal.io/stages/filters.smrf.html
.. _LAS writer: https://pdal.io/stages/writers.las.html

