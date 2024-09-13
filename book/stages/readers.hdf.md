(readers-hdf)=

# readers.hdf

The **HDF reader** reads data from files in the
[HDF5 format.](https://www.hdfgroup.org/solutions/hdf5/)
You must explicitly specify a mapping of HDF datasets to PDAL
dimensions using the dimensions parameter. ALL dimensions must
be scalars and be of the same length. Compound types are not
supported at this time.

```{eval-rst}
.. plugin::
```

```{eval-rst}
.. streamable::
```

## Example

This example reads from the Autzen HDF example with all dimension
properly mapped and then outputs a LAS file.

```json
[
    {
        "type": "readers.hdf",
        "filename": "test/data/hdf/autzen.h5",
        "dimensions":
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
        "filename": "output.las",
        "scale_x": 1.0e-5,
        "scale_y": 1.0e-5,
        "scale_z": 1.0e-5,
        "offset_x": "auto",
        "offset_y": "auto",
        "offset_z": "auto"
    }
]
```

:::{note}
All dimensions must be simple numeric HDF datasets with
equal lengths. Compound types, enum types, string types,
etc. are not supported.
:::

:::{warning}
The HDF reader does not set an SRS.
:::

## Common Use Cases

A possible use case for this driver is reading NASA's [ICESat-2](https://icesat-2.gsfc.nasa.gov/) data.
This example reads the X, Y, and Z coordinates from the ICESat-2
[ATL03](https://icesat-2.gsfc.nasa.gov/sites/default/files/page_files/ICESat2_ATL03_ATBD_r002.pdf) format and converts them into a LAS file.

:::{note}
ICESat-2 data use [EPSG:7912](https://epsg.io/7912). ICESat-2 Data products documentation can be found [here](https://icesat-2.gsfc.nasa.gov/science/data-products)
:::

```json
[
    {
        "type": "readers.hdf",
        "filename": "ATL03_20190906201911_10800413_002_01.h5",
        "dimensions":
        {
            "X" : "gt1l/heights/lon_ph",
            "Y" : "gt1l/heights/lat_ph",
            "Z" : "gt1l/heights/h_ph"
        }
    },
    {
        "type" : "writers.las",
        "filename": "output.las"
    }
]
```

## Options

```{eval-rst}
.. include:: reader_opts.rst
```

dimensions

: A JSON map with PDAL dimension names as the keys and HDF dataset paths as the values.
