.. _info_command:

********************************************************************************
info
********************************************************************************

Displays information about a point cloud file, such as:

* basic properties (extents, number of points, point format)
* coordinate reference system
* additional metadata
* summary statistics about the points
* the plain text format should be reStructured text if possible to allow a user
  to retransform the output into whatever they want with ease

::

    $ pdal info <input>

::

  --input, -i               Input file name
  --all                     Dump statistics, schema and metadata
  --point, -p               Point to dump --point="1-5,10,100-200" (0 indexed)
  --query                   Return points in order of distance from the
      specified location (2D or 3D) --query Xcoord,Ycoord[,Zcoord][/count]
  --stats                   Dump stats on all points (reads entire dataset)
  --boundary                Compute a hexagonal hull/boundary of dataset
  --dimensions              Dimensions on which to compute statistics
  --enumerate               Dimensions whose values should be enumerated
  --schema                  Dump the schema
  --pipeline-serialization  Output filename for pipeline serialization
  --summary                 Dump summary of the info
  --metadata                Dump file metadata info
  --stdin, -s               Read a pipeline file from standard input

If no options are provided, ``--stats`` is assumed.

Example 1:
^^^^^^^^^^^^

::

    $ pdal info  test/data/las/1.2-with-color.las \
        --query="636601.87, 849018.59, 425.10"
    {
      "0":
      {
        "Blue": 134,
        "Classssification": 1,
        "EdgeOfFlightLine": 0,
        "GpsTime": 245383.38808001476,
        "Green": 104,
        "Intensity": 124,
        "NumberOfReturns": 1,
        "PointSourceId": 7326,
        "Red": 134,
        "ReturnNumber": 1,
        "ScanAngleRank": -4,
        "ScanDirectionFlag": 1,
        "UserData": 126,
        "X": 636601.87,
        "Y": 849018.59999999998,
        "Z": 425.10000000000002
      },
      "1":
      {
        "Blue": 134,
        "Classification": 2,
        "EdgeOfFlightLine": 0,
        "GpsTime": 246099.17323102333,
        "Green": 106,
        "Intensity": 153,
        "NumberOfReturns": 1,
        "PointSourceId": 7327,
        "Red": 143,
        "ReturnNumber": 1,
        "ScanAngleRank": -10,
        "ScanDirectionFlag": 1,
        "UserData": 126,
        "X": 636606.76000000001,
        "Y": 849053.94000000006,
        "Z": 425.88999999999999
      },
      ...

Example 2:
^^^^^^^^^^

::

    $ pdal info test/data/1.2-with-color.las -p 0-10
    {
      "filename": "../../test/data/las/1.2-with-color.las",
      "pdal_version": "PDAL 1.0.0.b1 (116d7d) with GeoTIFF 1.4.1 GDAL 1.11.2 LASzip 2.2.0",
      "points":
      {
        "point":
        [
          {
            "Blue": 88,
            "Classification": 1,
            "EdgeOfFlightLine": 0,
            "GpsTime": 245380.78254962614,
            "Green": 77,
            "Intensity": 143,
            "NumberOfReturns": 1,
            "PointId": 0,
            "PointSourceId": 7326,
            "Red": 68,
            "ReturnNumber": 1,
            "ScanAngleRank": -9,
            "ScanDirectionFlag": 1,
            "UserData": 132,
            "X": 637012.23999999999,
            "Y": 849028.31000000006,
            "Z": 431.66000000000003
          },
          {
            "Blue": 68,
            "Classification": 1,
            "EdgeOfFlightLine": 0,
            "GpsTime": 245381.45279923646,
            "Green": 66,
            "Intensity": 18,
            "NumberOfReturns": 2,
            "PointId": 1,
            "PointSourceId": 7326,
            "Red": 54,
            "ReturnNumber": 1,
            "ScanAngleRank": -11,
            "ScanDirectionFlag": 1,
            "UserData": 128,
            "X": 636896.32999999996,
            "Y": 849087.70000000007,
            "Z": 446.38999999999999
          },
          ...

