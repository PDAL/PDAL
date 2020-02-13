.. _readers.hdf:

readers.hdf
===============

The **HDF reader** reads from files in the HDF5 format. See the
TODO for more information.

.. plugin::

.. streamable::

Example
-------
This example reads from the Autzen HDF example with all dimension
properly mapped and then outputs a LAS file.

.. code-block:: json

    [
        {
            "type": "readers.hdf",
            "filename": "test/data/hdf/autzen.h5",
            "map":
            {
                "X" : "autzen/X",
                "Y" : "autzen/Y",
                "Z" : "autzen/Z",
                "Red" : "autzen/Red",
                "Blue" : "autzen/Blue",
                "Green" : "autzen/Green",
                "Classification" : "autzen/Classification",
                "EdgeOfFlightLine" : "autzen/EdgeOfFlightLine",
                "GpsTime" : "autzen/GpsTime",
                "Intensity" : "autzen/Intensity",
                "NumberOfReturns" : "autzen/NumberOfReturns",
                "PointSourceId" : "autzen/PointSourceId",
                "ReturnNumber" : "autzen/ReturnNumber",
                "ScanAngleRank" : "autzen/ScanAngleRank",
                "ScanDirectionFlag" : "autzen/ScanDirectionFlag",
                "UserData" : "autzen/UserData"
            }
        },
        {
            "type" : "writers.las",
            "filename": "output.las"
        }
    ]


This example reads the X, Y, and Z coordinates from the ICESAT2
ATL03 format and converts them into a LAS file.

.. note::
    All dimensions must be simple numeric HDF datasets with
    equal lengths. Compound types, enum types, string types,
    etc. are not supported.


.. code-block:: json

    [
        { 
            "type": "readers.hdf", 
            "filename": "ATL03_20190906201911_10800413_002_01.h5",  
            "map": 
            { 
                "X" : "gt1l/heights/lon_ph", 
                "Y" : "gt1l/heights/lat_ph", 
                "Z" : "gt1l/heights/h_ph", 
            } 
        }, 
        { 
            "type" : "writers.las", 
            "filename": "output.las" 
        } 
    ] 


`ICESAT2 Data products Documentation <https://icesat-2.gsfc.nasa.gov/science/data-products>`_
~                                                                                              
~             
Options
-------

filename
  File to read from [Required]

.. include:: reader_opts.rst

map
  A JSON map with PDAL dimension names as the keys and HDF dataset paths as the values.

metadata
  TBD

