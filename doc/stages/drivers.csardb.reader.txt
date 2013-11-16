.. _drivers.csardb.reader:

drivers.csardb.reader
=====================

The **CSAR database reader** supports reading from `CARIS point cloud databases`_.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.text.writer">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="drivers.csardb.reader">
        <Option name="connection">username/password@hostname%dbname%boid</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

connection
  Database connection string formated as: "username/password@hostname%dbname%boid" [Required] 



.. _CARIS point cloud databases: http://www.caris.com/products/bathydatabase/base-manager/indepth.cfm
  
