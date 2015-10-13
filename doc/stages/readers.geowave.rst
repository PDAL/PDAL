.. _readers.geowave:

readers.geowave
===============================================================================

The **GeoWave reader** uses `GeoWave`_ to read from Accumulo.  GeoWave entries
are stored using EPSG:4326.  Instructions for configuring the GeoWave plugin
can be found `here`_

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="readers.geowave">
        <Option name="zookeeper_url">zookeeper1:2181,zookeeper2:2181,zookeeper3:2181</Option>
        <Option name="instance_name">GeoWave</Option>
        <Option name="username">user</Option>
        <Option name="password">pass</Option>
        <Option name="table_namespace">PDAL_Table</Option>
        <Option name="feature_type_name">PDAL_Point</Option>
        <Option name="data_adapter">FeatureCollectionDataAdapter</Option>
        <Option name="points_per_entry">5000u</Option>
        <Option name="bounds">([0,1000000],[0,1000000],[0,100])</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

zookeeper_url
  The comma-delimited URLs for all zookeeper servers, this will be directly used to instantiate a ZookeeperInstance. [Required]

instance_name
  the zookeeper instance name, this will be directly used to instantiate a ZookeeperInstance. [Required]

username
  The username for the account to establish an Accumulo connector. [Required]

password
  The password for the account to establish an Accumulo connector. [Required]

table_namespace
  The table name to be used when interacting with GeoWave. [Required]

feature_type_name
  The feature type name to be used when ineracting GeoWave. [Default: PDAL_Point]

data_adapter
  FeatureCollectionDataAdapter stores multiple points per Accumulo entry. FeatureDataAdapter stores a single point per Accumulo entry. [Default: FeatureCollectionDataAdapter]

points_per_entry
  Sets the maximum number of points per Accumulo entry when using FeatureCollectionDataAdapter. [Default: 5000u]

bounds
  The extent of the bounding rectangle to use to query points, expressed as a string, eg: “([xmin,xmax],[ymin,ymax],[zmin,zmax])”. [Default: unit cube]


.. _GeoWave: https://ngageoint.github.io/geowave/
.. _here: https://ngageoint.github.io/geowave/documentation.html#jace-jni-proxies-2

