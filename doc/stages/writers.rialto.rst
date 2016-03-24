.. _writers.rialto:

writers.rialto
==============

The **RialtoWriter** supports writing to `Rialto-formatted
tiles <http://lists.osgeo.org/pipermail/pointdown/2015-February/000001.html>`__.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.las",
          "filename":"inputfile.las"
        },
        {
          "type":"filters.reprojection",
          "out_srs":"EPSG:4326"
        }
        {
          "type":"writers.rialto",
          "max_levels":"18",
          "overwrite":"true",
          "filename":"outputfile.ria"
        }
      ]
    }

Options
-------

filename
  The **directory** to stage the Rialto tiles in. An exception will be thrown
  if the directory exists, unless the overwrite option is set to true (see
  below). [Required]

max_levels
  The maximum number of levels in the quadtree. Each rectangular node at level
  L reduces to 4 equally-sized nodes at level L+1. Each tile at level N-1
  contains 1/4 of the points contained in the level N nodes. [Default: 16]

overwrite
  Delete the target directory prior to writing results? [Default: false]

