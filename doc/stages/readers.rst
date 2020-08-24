.. _readers:

Readers
=======

Readers provide :ref:`dimensions` to :ref:`pipeline`. PDAL attempts to
normalize common dimension types, like X, Y, Z, or Intensity, which are often
found in LiDAR point clouds. Not all dimension types need to be fixed, however.
Database drivers typically return unstructured lists of dimensions.  A reader
might provide a simple file type, like :ref:`readers.text`, a complex database
like :ref:`readers.oci`, or a network service like :ref:`readers.ept`.


.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   readers.bpf
   readers.buffer
   readers.ept
   readers.e57
   readers.faux
   readers.gdal
   readers.geowave
   readers.hdf
   readers.i3s
   readers.ilvis2
   readers.las
   readers.matlab
   readers.memoryview
   readers.mbio
   readers.mrsid
   readers.nitf
   readers.numpy
   readers.oci
   readers.optech
   readers.pcd
   readers.pgpointcloud
   readers.ply
   readers.pts
   readers.qfit
   readers.rdb
   readers.rxp
   readers.sbet
   readers.sqlite
   readers.slpk
   readers.terrasolid
   readers.text
   readers.tiledb
   readers.tindex

:ref:`readers.bpf`
    Read BPF files encoded as version 1, 2, or 3. BPF is an NGA specification
    for point cloud data.

:ref:`readers.buffer`
    Special stage that allows you to read data from your own PointView rather
    than fetching data from a specific reader.

:ref:`readers.ept`
    Used for reading `Entwine Point Tile <https://entwine.io>`__ format.

:ref:`readers.e57`
    Read point clouds in the E57 format.

:ref:`readers.faux`
    Used for testing pipelines. It does not read from a file or database, but
    generates synthetic data to feed into the pipeline.

:ref:`readers.gdal`
    Read GDAL readable raster data sources as point clouds.

:ref:`readers.geowave`
    Read point cloud data from Accumulo.

:ref:`readers.hdf`
   Read data from files in the HDF5 format.

:ref:`readers.i3s`
    Read data stored in the Esri I3S format.  The data is read from an
    appropriate server.

:ref:`readers.ilvis2`
    Read from files in the ILVIS2 format.

:ref:`readers.las`
    Read ASPRS LAS versions 1.0 - 1.4. Does not support point formats
    containing waveform data. LASzip support is also enabled through this
    driver if LASzip  or LAZperf are found during compilation.

:ref:`readers.matlab`
    Read point cloud data from MATLAB .mat files where dimensions are stored as
    arrays in a MATLAB struct.

:ref:`readers.mbio`
    Read sonar bathymetry data from formats supported by the MB-System library.

:ref:`readers.memoryview`
    Read data from memory where dimension data is arranged in rows.  For
    use only with the PDAL API.

:ref:`readers.mrsid`
    Read data compressed by the MrSID 4.0 LiDAR Compressor. Requires the
    LizardTech Lidar_DSDK.

:ref:`readers.nitf`
    Read point cloud data (LAS or LAZ) wrapped in NITF 2.1 files.

:ref:`readers.numpy`
    Read point cloud data from Numpy ``.npy`` files.

:ref:`readers.oci`
    Read data from Oracle point cloud databases.

:ref:`readers.optech`
    Read Optech Corrected Sensor Data (.csd) files.

:ref:`readers.pcd`
    Read files in the PCD format.

:ref:`readers.pgpointcloud`
    Read point cloud data from a PostgreSQL database with the PostgreSQL
    Pointcloud extension enabled.

:ref:`readers.ply`
    Read points and vertices from either ASCII or binary PLY files.

:ref:`readers.pts`
    Read data from Leica Cyclone PTS files.

:ref:`readers.qfit`
    Read data in the QFIT format originated for NASA's Airborne Topographic
    Mapper project.

:ref:`readers.rxp`
    Read data in the RXP format, the in-house streaming format used by RIEGL.
    The reader requires a copy of RiVLib during compilation.

:ref:`readers.rdb`
    Read data in the RDB format, the in-house database format used by RIEGL.
    The reader requires a copy of rdblib during compilation and usage.

:ref:`readers.sbet`
    Read the SBET format.

:ref:`readers.sqlite`
    Read data stored in a SQLite database.

:ref:`readers.slpk`
    Read data stored in an Esri SLPK file.

:ref:`readers.terrasolid`
    TerraSolid Reader

:ref:`readers.text`
    Read point clouds from ASCII text files.

:ref:`readers.tiledb`
    Read point cloud data from a TileDB instance.

:ref:`readers.tindex`
    The tindex (tile index) reader allows you to automatically merge and query
    data described in tile index files that have been generated using the PDAL
    tindex command.
