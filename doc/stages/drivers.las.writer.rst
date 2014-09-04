.. _drivers.las.writer:

drivers.las.writer
==================

The **LAS Writer** supports writing to `LAS format`_ files, the standard interchange file format for LIDAR data.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">outputfile.las</Option>
      
      <Reader type="drivers.las.reader">
        <Option name="filename">inputfile.las</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  LAS file to read [Required] 

software_id
  String identifying the software that created this LAS file. [Default: "libLAS 1.2"]

creation_doy
  Number of the day of the year (January 1 == 0, Dec 31 == 365) this file is being created. [Default: 0]
  
creation_year
  Year (Gregorian) this file is being created. [Default: 0]
  
format
  Controls whether information about color and time are stored with the point information in the LAS file. [Default: 3]
  
  * 0 == no color or time stored
  * 1 == time is stored
  * 2 == color is stored
  * 3 == color and time are stored 
  
system_id
  String identifying the system that created this LAS file. [Default: "libLAS"]
  
header_padding
  The number of bytes between the variable-length header information and the point data itself. Extra header padding allows extra headers to be written later without re-writing the whole file [Default: 0]

a_srs
  The spatial reference system of the file to be written. Can be an EPSG string (eg "EPSG:268910") or a WKT string. [Default: Not set]
  
compression
  Apply compressed to the LAS file? [Default: false]
  
filesource_id
  The file source id number to use for this file (a value between 1 and 65535) [Default: 0]

discard_high_return_numbers
  If true, discard all points with a return number greater than five, and clamp NumberOfReturns to five. [Default: false]


.. _LAS format: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
  
