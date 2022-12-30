.. _readers.stac:

readers.stac
============

`Spatio Temporal Access Catalog (STAC)`_ is a common language to describe geospatial
information, so it can more easily be worked with, indexed, and discovered. The STAC
reader will read Catalogs and Features. For Catalogs, the reader will iterate through
items available in the Links key, creating a list of reads to accomplish.

.. embed::

Example
--------------------------------------------------------------------------------

.. code-block:: json

    [
        {
            "type": "readers.stac",
            "filename": "https://s3-us-west-2.amazonaws.com/usgs-lidar-stac/ept/catalog.json",
            "reader_args": [{"type": "readers.ept", "resolution": 100}],
            "item_ids": ["MD_GoldenBeach_2012"],
            "catalog_ids": ["3dep"],
            "properties": { "pc:type": ["lidar", "sonar"], "pc:encoding": "ept" },
            "asset_name": "ept.json",
            "date_ranges": [
                [
                    "2022-11-11T0:00:0Z",
                    "2022-11-30T0:00:0Z"
                ]
            ],
            "validate_schema": true
        }
    ]

.. code-block:: bash

    pdal info --input https://s3-us-west-2.amazonaws.com/usgs-lidar-stac/ept/MD_GoldenBeach_2012.json \
        --driver readers.stac --asset_name ept.json --summary

Options
--------------------------------------------------------------------------------
filename
    STAC endpoint, local or remote, that corresponds to a Catalog or Feature.

asset_names
    The list of asset names that should be looked at to find the source data.
    The default is 'data'.

date_ranges
    A list of date ranges to prune Features by.
    Example: ``--readers.stac.date_ranges '[["2022-11-11T0:00:0Z","2022-11-30T0:00:0Z"],...]'``

bounds
    Bounds to prune Features by.
    Form: ``([minx,maxx],[miny,maxy],[minz,maxz])``
    Example: ``--readers.stac.bounds '([-79.0,-74.0],[38.0,39.0])'``

item_ids
    List of `Regular Expression`_ strings to prune STAC Item IDs by.
    Example: ``--readers.stac.item_ids '["MD_GoldenBeach_2012", "USGS_LPC\\w{0,}"]'``

catalog_ids
    List of `Regular Expression`_ strings to prune STAC Catalog IDs by.
    Root catalog IDs are always included in the list.
    Example: ``--readers.stac.catalog_ids '["3dep-\*", "USGS"]'``

validate_schema
    Boolean value determining if the reader should validate the supplied STAC as
    it's being read using JSON schema and the publicly available STAC schemas and
    list of STAC extension schemas.

properties
    A key value mapping (JSON) of properties and the desired values to prune
    Features by. Different keys will be AND'd together, and list of values will
    OR'd together.
    Example: ``--readers.stac.properties'{"pc:type":["lidar","sonar"],"pc:encoding":"ept"}'``
    In this example, a Feature must have a `pc:type` key with values of either
    `lidar` or `sonar`, and a `pc:encoding` key with a value of `ept`.

reader_args
    A list of JSON objects with keys of reader options and the values to pass through.
    These will be in the exact same form as a Pipeline Stage object minus the filename.

    Exmaple:

.. code-block:: bash

    -readers.stac.reader_args \
    '[{"type": "readers.ept", "resolution": 100}, {"type": "readers.las", "nosrs": true}]'


catalog_schema_url
    URL of JSON schema you'd like to use for JSON schema validation of STAC Catalogs.

feature_schema_url
    URL of JSON schema you'd like to use for JSON schema validation of STAC Items/Features.


.. _Spatio Temporal Access Catalog (STAC): https://stacspec.org/en
.. _Regular Expression: https://en.cppreference.com/w/cpp/regex
