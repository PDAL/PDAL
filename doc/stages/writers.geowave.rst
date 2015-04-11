.. _writers.geowave:

writers.geowave
============

The **GeoWave writer** uses `GeoWave`_ to write to Accumulo.  GeoWave entries are stored using EPSG:4326.  Instructions for configuring the GeoWave plugin can be found `here`_


Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.geowave">
      <Option name="zookeeperUrl">zookeeper1:2181,zookeeper2:2181,zookeeper3:2181</Option>
      <Option name="instanceName">GeoWave</Option>
      <Option name="username">user</Option>
      <Option name="password">pass</Option>
      <Option name="tableNamespace">PDAL_Table</Option>
      <Option name="featureTypeName">PDAL_Point</Option>
      <Option name="dataAdapter">FeatureCollectionDataAdapter</Option>
      <Option name="pointsPerEntry">5000u</Option>
      <Filter type="filters.reprojection">
        <Option name="out_srs">EPSG:4326</Option>
        <Reader type="readers.qfit">
          <Option name="filename">qfitfile.qi</Option>
          <Option name="flip_coordinates">false</Option>
          <Option name="scale_z">1.0</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

Options
-------

zookeeperUrl
  The comma-delimited URLs for all zookeeper servers, this will be directly used to instantiate a ZookeeperInstance. [Required]

instanceName
  The zookeeper instance name, this will be directly used to instantiate a ZookeeperInstance. [Required]

username
  The username for the account to establish an Accumulo connector. [Required]

password
  The password for the account to establish an Accumulo connector. [Required]

tableNamespace
  The table name to be used when interacting with GeoWave. [Required]

featureTypeName
  The feature type name to be used when ineracting GeoWave. [Default: PDAL_Point]

dataAdapter
  FeatureCollectionDataAdapter stores multiple points per Accumulo entry. FeatureDataAdapter stores a single point per Accumulo entry. [Default: FeatureCollectionDataAdapter]

pointsPerEntry
  Sets the maximum number of points per Accumulo entry when using FeatureCollectionDataAdapter. [Default: 5000u]  


.. _GeoWave: https://ngageoint.github.io/geowave/
.. _here: https://ngageoint.github.io/geowave/documentation.html#jace-jni-proxies-2

