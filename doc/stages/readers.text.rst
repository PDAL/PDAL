.. _readers.text:

readers.text
============

The **text reader** reads data from ASCII text files.  Each point is
represented in the file as a single line.  Each line is expected to be divided
into a number of fields by a separator.  Each field represents a value for
a point's dimension.  Each value needs to be `formatted`_ properly for
C++ language double-precision values.

The text reader expects a header line to 1) indicate the separator character
for the fields and 2) name the dimension for each field in the points.  Any
single non-alphanumeric character can be used as a separator.  The header line
separator can be overridden by the 'separator' option (see below).
Each line in the file must contain the same number of fields as indicated by
dimension names in the header.  Spaces are generally ignored in the input
unless used as a separator.  When a space character is used as a separator,
any number of consecutive spaces are treated as single space.

Blank lines after the header line are ignored.

.. embed::

.. streamable::

Example Input File
------------------

This input file contains X, Y and Z value for 10 points.

::

    X,Y,Z
    289814.15,4320978.61,170.76
    289814.64,4320978.84,170.76
    289815.12,4320979.06,170.75
    289815.60,4320979.28,170.74
    289816.08,4320979.50,170.68
    289816.56,4320979.71,170.66
    289817.03,4320979.92,170.63
    289817.53,4320980.16,170.62
    289818.01,4320980.38,170.61
    289818.50,4320980.59,170.58

Example Pipeline
----------------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.text",
          "filename":"inputfile.txt"
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
  text file to read [Required]

separator
  Separator character to override that found in header line.

count
  Maximum number of points to read [Optional]

.. _formatted: http://en.cppreference.com/w/cpp/string/basic_string/stof
