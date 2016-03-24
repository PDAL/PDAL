.. _readers.ply:

readers.ply
===========

The **ply reader** reads the `polygon file format`_, a common file format for storing three dimensional models.
The `rply library`_ is included with the PDAL source, so there are no external dependencies.

The ply reader can read ASCII and binary ply files.


Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.ply",
          "filename":"inputfile.ply"
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }


Options
-------

filename
  ply file to read [Required]



.. _polygon file format: http://paulbourke.net/dataformats/ply/
.. _rply library: http://w3.impa.br/~diego/software/rply/
