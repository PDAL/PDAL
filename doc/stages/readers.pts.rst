.. _readers.pts:

readers.pts
============

The **PTS reader** reads data from Leica Cyclone PTS files. It is
not very sophisticated.


Example Pipeline
----------------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.pts",
          "filename":"test.pts"
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

