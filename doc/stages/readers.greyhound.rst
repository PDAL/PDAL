.. _readers.greyhound:

readers.greyhound
=================

The **Greyhound Reader** allows you to read point data from a `Greyhound`_ server.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.greyhound",
          "url":"data.greyhound.io/resource/autzen"
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }


Options
-------

url
  Greyhound server URL string. [Required]



.. _Greyhound: https://github.com/hobu/greyhound
