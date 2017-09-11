.. _readers.mbio:

readers.mbio
============

The mbio reader allows sonar bathymetry data to be read into PDAL and
treated as data collected using LIDAR sources.  PDAL uses the `MB-System`_
library to read the data and therefore supports `all formats`_ supported by
that library.  Some common sonar systems are NOT supported by MB-System,
notably Kongsberg, Reson and Norbit.  The mbio reader reads each "beam"
of data after averaging and processing by the MB-System software and stores
the values for the dimensions 'X', 'Y', 'Z' and 'Amplitude'.  X and Y use
longitude and latitude for units and the Z values are in meters (negative,
being below the surface).  Units for 'Amplitude' is not specified and may
vary.

.. plugin::

.. streamable::


Example
-------

This reads beams from a sonar data file and writes points to a LAS file.

.. code-block:: json

    {
      "pipeline":[
        {
          "type" : "readers.mbio",
          "filename" : "shipdata.m57",
          "format" : "MBF_EM3000RAW"
        },
        {
          "type":"writers.las",
          "filename":"outputfile.las"
        }
      ]
    }


Options
-------

filename
  Filename to read from [Required]

format
  Name of number of format of file being read.  See MB-System documentation
  for a list of `all formats`_. [Required]


.. _MB-System: http://www.ldeo.columbia.edu/res/pi/MB-System/

.. _all formats: https://www.ldeo.columbia.edu/res/pi/MB-System/html/mbio.html#lbAI

count
  Maximum number of points to read [Optional]
