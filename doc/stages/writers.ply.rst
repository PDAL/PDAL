.. _writers.ply:

writers.ply
===========

The **ply writer** writes the `polygon file format`_, a common file format for storing three dimensional models.
The `rply library`_ is included with the PDAL source, so there are no external dependencies.

Use the ``storage_mode`` option to choose the type of ply file to write.
You can choose from:

- ``default``: write a binary ply file using your host's byte ordering.
  If you do not specify a ``storage_mode``, this is the default.
- ``ascii``: write an ascii file (warning: these can be HUGE).
- ``little endian``: write a binary ply file with little endian byte ordering.
- ``big endian``: write a binary ply file with big endian byte ordering.


Example
-------


.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.pcd",
          "filename":"inputfile.pcd"
        },
        {
          "type":"writers.ply",
          "storage_mode":"little endian",
          "filename":"outputfile.ply"
        }
      ]
    }


Options
-------

filename
  ply file to write [Required]

storage_mode
  Type of ply file to write [default: host-ordered binary]


.. _polygon file format: http://paulbourke.net/dataformats/ply/
.. _rply library: http://w3.impa.br/~diego/software/rply/
