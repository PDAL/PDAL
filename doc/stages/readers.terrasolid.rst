.. _readers.terrasolid:

readers.terrasolid
==================

The **Terrasolid Reader** loads points from `Terrasolid`_ files (.bin).
It supports boths Terrasolid format 1 and format 2.

Example
-------

.. code-block:: json

  [
      {
          "type":"readers.terrasolid",
          "filename":"autzen.bin"
      },
      {
          "type":"writers.las",
          "filename":"output.las"
      }
  ]

Options
-------

filename
  Input file name [Required]

.. include:: reader_opts.rst

.. _Terrasolid: https://www.terrasolid.com/home.php
