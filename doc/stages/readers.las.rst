.. _readers.las:

readers.las
===========

The **LAS Reader** supports reading from `LAS format`_ files, the standard
interchange format for LIDAR data.  The reader does NOT support point formats
containing waveform data (4, 5, 9 and 10).

The reader will accept version 1.4 LAS files and will interpret the extra bytes
VLR (User ID: LASF_Spec, Record ID: 4) if present.  Two dimensions are
created for each
extra byte field with types 11 - 20, "0" and "1" being appended to the field
name to create the dimension names. Three dimensions are created for each
extra byte field with types 21-30, "0", "1" and "2" being appended to the field
name to create the dimension names.  Field type 0 is understood, but the data
associated with field type 0 is ignored (no PDAL dimension is created).  The
presence of this VLR overrides the **extra_dims** option.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="readers.las">
        <Option name="filename">inputfile.las</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  LAS file to read [Required] 

extra_dims
  Extra dimensions to be read as part of each point beyond those specified by
  the LAS point format.  The format of the option is
  <dimension_name>=<type>, ... where type is one of:
      int8, int16, int32, int64, uint8, uint16, uint32, uint64, float, double
  '_t' may be added to any of the type names as well (e.g., uint32_t)

.. _LAS format: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
  
