.. _writers.text:

writers.text
============

The **text writer** writes out to a text file. This is useful for debugging or
getting smaller files into an easily parseable format.  The text writer
supports both `GeoJSON`_ and `CSV`_ output.


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
          "type":"writers.text",
          "format":"geojson",
          "order":"X,Y,Z",
          "keep_unspecified":"false",
          "filename":"outputfile.txt"
        }
      ]
    }

Options
-------

filename
  File to write to, or "STDOUT" to write to standard out [Required]

format
  Output format to use. One of "geojson" or "csv". [Default: **csv**]

order
  Comma-separated list of dimension names, giving the desired column order in the output file, for example "X,Y,Z,Red,Green,Blue". [Default: none]

keep_unspecified
  Should we output any fields that are not specified in the dimension order? [Default: **true**]

jscallback
  When producing GeoJSON, the callback allows you to wrap the data in a function, so the output can be evaluated in a <script> tag.

quote_header
  When producing CSV, should the column header named by quoted? [Default: **true**]

newline
  When producing CSV, what newline character should be used? (For Windows, "\\r\\n" is common.) [Default: **\\n**]

delimiter
  When producing CSV, what character to use as a delimiter? [Default: **,**]


.. _GeoJSON: http://geojson.org
.. _CSV: http://en.wikipedia.org/wiki/Comma-separated_values

