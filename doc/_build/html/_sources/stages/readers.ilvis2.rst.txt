.. _readers.ilvis2:

readers.ilvis2
===============

The **ILVIS2 reader** read from files in the ILVIS2 format. See the
`product spec <https://nsidc.org/data/ilvis2>`_ for more information.

.. figure:: readers.ilvis2.metadata.png

    Dimensions provided by the ILVIS2 reader

.. embed::

.. streamable::

Example
-------

.. code-block:: json

  [
      {
          "type":"readers.ilvis2",
          "filename":"ILVIS2_GL2009_0414_R1401_042504.TXT",
          "metadata":"ILVIS2_GL2009_0414_R1401_042504.xml"
      },
      {
          "type":"writers.las",
          "filename":"outputfile.las"
      }
  ]

Options
-------

filename
  File to read from [Required]

.. include:: reader_opts.rst

mapping
  Which ILVIS2 field type to map to X, Y, Z dimensions
  'LOW', 'CENTROID', or 'HIGH' [Default: 'CENTROID']

metadata
  XML metadata file to coincidentally read [Optional]

