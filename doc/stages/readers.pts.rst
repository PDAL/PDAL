.. _readers.pts:

readers.pts
============

The **PTS reader** reads data from Leica Cyclone PTS files.  It infers
dimensions from points stored in a text file.

.. embed::


Example Pipeline
----------------

.. code-block:: json

  [
      {
          "type":"readers.pts",
          "filename":"test.pts"
      },
      {
          "type":"writers.text",
          "filename":"outputfile.txt"
      }
  ]

Options
-------

filename
  File to read. [Required]

.. include:: reader_opts.rst

