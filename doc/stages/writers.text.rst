.. _writers.text:

writers.text
============

The **text writer** writes out to a text file. This is useful for debugging or
getting smaller files into an easily parseable format.  The text writer
supports both `GeoJSON`_ and `CSV`_ output.


.. embed::

.. streamable::

Example
-------

.. code-block:: json

  [
      {
          "type":"readers.las",
          "filename":"inputfile.las"
      },
      {
          "type":"writers.text",
          "format":"geojson",
          "order":"X,Y,Z",
          "keep_unspecified":"false",
          "filename":"outputfile.txt"
      }
  ]

Options
-------

filename
  File to write to, or "STDOUT" to write to standard out [Required]

format
  Output format to use. One of ``geojson`` or ``csv``. [Default: "csv"]

_`precision`
  Decimal Precision for output of values. This can be overridden for
  individual dimensions using the order option. [Default: 3]

_`order`
  Comma-separated list of dimension names in the desired output order.
  For example "X,Y,Z,Red,Green,Blue". Dimension names
  can optionally be followed with a colon (':') and an integer to indicate the
  precision to use for output. Ex: "X:3, Y:5,Z:0" If no precision is specified
  the value provided with the precision_ option is used. [Default: none]

keep_unspecified
  If true, writes all dimensions.  Dimensions specified with the order_
  option precede those not specified. [Default: **true**]

jscallback
  When producing GeoJSON, the callback allows you to wrap the data in
  a function, so the output can be evaluated in a <script> tag.

quote_header
  When producing CSV, should the column header named by quoted?
  [Default: true]

write_header
  Whether a header should be written. [Default: true]

newline
  When producing CSV, what newline character should be used? (For Windows,
  ``\\r\\n`` is common.) [Default: "\\n"]

delimiter
  When producing CSV, what character to use as a delimiter? [Default: ","]

.. include:: writer_opts.rst

.. _GeoJSON: http://geojson.org
.. _CSV: http://en.wikipedia.org/wiki/Comma-separated_values

