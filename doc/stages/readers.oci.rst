.. _readers.oci:

readers.oci
===========

The OCI reader is used to read data from `Oracle point cloud`_ databases.


Example
-------


.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.oci",
          "query":"SELECT \r\n            l.\"OBJ_ID\", l.\"BLK_ID\", l.\"BLK_EXTENT\", \r\n            l.\"BLK_DOMAIN\", l.\"PCBLK_MIN_RES\", \r\n            l.\"PCBLK_MAX_RES\", l.\"NUM_POINTS\",\r\n            l.\"NUM_UNSORTED_POINTS\", l.\"PT_SORT_DIM\", \r\n            l.\"POINTS\", b.cloud\r\n          FROM AUTZEN_BLOCKS l, AUTZEN_CLOUD b\r\n          WHERE l.obj_id = b.id and l.obj_id in (1,2)\r\n          ORDER BY l.obj_id",
          "connection":"grid/grid@localhost/orcl",
          "populate_pointsourceid":"true"
        },
        {
          "type":"writers.las",
          "filename":"outputfile.las"
        }
      ]
    }



Options
-------

connection
  Oracle connection string to connect to database, in the form "user/pass@host/instance" [Required]

query
  SELECT statement that returns an SDO_PC object as its first and only queried item [Required]

spatialreference
  Spatial reference system of the data being read. E.g. "EPSG:26910".

xml_schema_dump
  Filename to dump the XML schema to.

populate_pointsourceid
  Boolean value. If true, then add in a point cloud to every point read on the PointSourceId dimension. [Default: **false**]


.. _Oracle point cloud: http://docs.oracle.com/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm

