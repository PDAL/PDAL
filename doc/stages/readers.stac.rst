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
            "items": ["MD_GoldenBeach_2012"],
            "catalogs": ["3dep"],
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
    STAC endpoint, local or remote, that corresponds to a Catalog, Feature or ItemCollection.

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

items
    List of `Regular Expression`_ strings to prune STAC Item IDs by.
    Example: ``--readers.stac.items '["MD_GoldenBeach_2012", "USGS_LPC\\w{0,}"]'``

catalogs
    List of `Regular Expression`_ strings to prune STAC Catalog IDs by.
    Root catalog IDs are always included in the list.
    Example: ``--readers.stac.catalogs '["3dep-test", "USGS"]'``

collections
    List of `Regular Expression`_ strings to prune STAC Collection IDs by.
    This will filter by the `collections` key in a STAC Item and the `id` key
    of the STAC Collection.
    Example: ``--readers.stac.collections '["3dep-test", "USGS"]'``

validate_schema
    Boolean value determining if the reader should validate the supplied STAC as
    it's being read using JSON schema and the publicly available STAC schemas and
    list of STAC extension schemas.

properties
    A key value mapping (JSON) of properties and the desired values to prune
    Features by. Different keys will be AND'd together, and list of values will
    OR'd together.
    Example: ``--readers.stac.properties '{"pc:type":["lidar","sonar"],"pc:encoding":"ept"}'``
    In this example, a Feature must have a `pc:type` key with values of either
    `lidar` or `sonar`, and a `pc:encoding` key with a value of `ept`.

reader_args
    A list of JSON objects with keys of reader options and the values to pass through.
    These will be in the exact same form as a Pipeline Stage object minus the filename.

    Exmaple:

.. code-block:: bash

    --readers.stac.reader_args \
    '[{"type": "readers.ept", "resolution": 100}, {"type": "readers.las", "nosrs": true}]'


catalog_schema_url
    URL of JSON schema you'd like to use for JSON schema validation of STAC Catalogs.

collection_schema_url
    URL of JSON schema you'd like to use for JSON schema validation of STAC Collections.

feature_schema_url
    URL of JSON schema you'd like to use for JSON schema validation of STAC Items/Features.

Metadata
--------------------------------------------------------------------------------
Metadata outputs will include `ids` and `item_ids` for representings STAC Feature Ids,
as well as `catalog_ids` and `collection_ids` representing STAC Catalogs and Collections,
respectively.

.. code-block:: bash

    pdal info --summary --driver readers.stac \
    --readers.stac.asset_names 'ept.json' \
    --readers.stac.asset_names 'data' \
    ${PDAL_DIR}/test/data/stac/local_catalog/catalog.json

.. code-block:: json

    {
        "file_size": 1177,
        "filename": "/PDAL_DIR/test/data/stac/local_catalog/catalog.json",
        "now": "2023-08-07T15:48:59-0500",
        "pdal_version": "2.6.0 (git-version: 54be24)",
        "reader": "readers.stac",
        "summary":
        {
            "bounds":
            {
                "maxx": 637179.22,
                "maxy": 5740737,
                "maxz": 1069,
                "minx": -10543360,
                "miny": 848935.2,
                "minz": -22
            },
            "dimensions": "ClassFlags, Classification, EdgeOfFlightLine, GpsTime, Intensity, NumberOfReturns, PointSourceId, ReturnNumber, ScanAngleRank, ScanChannel, ScanDirectionFlag, UserData, X, Y, Z, OriginId, Red, Green, Blue",
            "metadata":
            {
                "catalog_ids":
                [
                    "3dep"
                ],
                "collection_ids":
                [
                    "usgs-test"
                ],
                "ids":
                [
                    "IA_SouthCentral_1_2020",
                    "MI_Charlevoix_Islands_TL_2018",
                    "MD_GoldenBeach_2012",
                    "Autzen Trim"
                ],
                "item_ids":
                [
                    "IA_SouthCentral_1_2020",
                    "MI_Charlevoix_Islands_TL_2018",
                    "MD_GoldenBeach_2012",
                    "Autzen Trim"
                ]
            },
            "num_points": 44851411750
        }
    }

Curl Timeouts
--------------------------------------------------------------------------------
STAC reader, and PDAL as a whole, rely on curl for external requests. The curl
requests default to a timeout of 5s. If your requests are failing, it could be
because the timeout is too short. You can set `CURL_TIMEOUT` in your environment
to get around this.

To debug your requests to make sure that the timeout is the problem, set `VERBOSE=1`
in your environment before running your PDAL task.

.. code-block:: bash

    VERBOSE=1 CURL_TIMEOUT=30 \
    pdal info --summary --driver readers.stac \
    --readers.stac.asset_names 'ept.json' \
    --readers.stac.asset_names 'data' \
    ${PDAL_DIR}/test/data/stac/local_catalog/catalog.json


.. _Spatio Temporal Access Catalog (STAC): https://stacspec.org/en
.. _Regular Expression: https://en.cppreference.com/w/cpp/regex
