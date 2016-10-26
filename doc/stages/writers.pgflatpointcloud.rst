.. _writers.pgflatpointcloud:

writers.pgflatpointcloud
====================

The **PostgreSQL Flat PointCloud Writer** allows you to write to PostgreSQL database to a normal flat table.

The flat table can then be used to create blocks for the Point Cloud extension
but this is something out of the scope of this writer.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.las",
          "filename":"inputfile.las",
          "spatialreference":"EPSG:26916"
        },
        {
          "type":"writers.pgflatpointcloud",
          "connection":"host='localhost' dbname='lidar' user='pramsey'",
          "table":"example"
        }
      ]
    }

Options
-------

connection
  PostgreSQL connection string. In the form *"host=hostname dbname=database user=username password=pw port=5432"* [Required]

table
  Database table to write to. [Required]

schema
  Database schema to write to. [Default: **public**]

overwrite
  To drop the table before writing set to 'true'. To append to the table set to 'false'. [Default: **true**]

output_dims
  If specified, limits the dimensions written for each point.  Dimensions
  are listed by name and separated by commas.
..
