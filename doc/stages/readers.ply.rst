.. _readers.pcd:

readers.ply
===========

The **ply reader** reads the `polygon file format`_, a common file format for storing three dimensional models.
The `rply library`_ is included with the PDAL source, so there are no external dependencies.

The ply reader can read ASCII and binary ply files.


Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="readers.ply">
        <Option name="filename">inputfile.ply</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  ply file to read [Required]



.. _polygon file format: http://paulbourke.net/dataformats/ply/
.. _rply library: http://w3.impa.br/~diego/software/rply/
