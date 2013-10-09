.. _drivers.oci.reader:

drivers.oci.reader
==================

The OCI reader is used to read data from `Oracle point cloud`_ databases.


Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">outputfile.las</Option>
      <Reader type="drivers.oci.reader">
        <Option name="query">
          SELECT 
            l."OBJ_ID", l."BLK_ID", l."BLK_EXTENT", 
            l."BLK_DOMAIN", l."PCBLK_MIN_RES", 
            l."PCBLK_MAX_RES", l."NUM_POINTS",
            l."NUM_UNSORTED_POINTS", l."PT_SORT_DIM", 
            l."POINTS", b.cloud
          FROM AUTZEN_BLOCKS l, AUTZEN_CLOUD b
          WHERE l.obj_id = b.id and l.obj_id in (1,2)
          ORDER BY l.obj_id
        </Option>
        <Option name="connection">
          grid/grid@localhost/orcl
        </Option>
        <Option name="populate_pointsourceid">
          true
        </Option>
      </Reader>
    </Writer>
  </Pipeline>


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

