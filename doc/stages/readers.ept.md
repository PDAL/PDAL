(readers.ept)=

# readers.ept

[Entwine Point Tile] (EPT) is a hierarchical octree-based point cloud format
suitable for real-time rendering and lossless archival.  [Entwine] is a
producer of this format.  The EPT Reader supports reading data from the
EPT format, including spatially accelerated queries and file reconstruction
queries.

Sample EPT datasets of hundreds of billions of points in size may be viewed
with [Potree].

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example

This example downloads a small area around the Statue of Liberty from the New York City data set (4.7 billion points) which can be viewed in its entirety in [Potree].

```json
[
   {
      "type": "readers.ept",
      "filename": "https://na-c.entwine.io/nyc/ept.json",
      "bounds": "([-8242669, -8242529], [4966549, 4966674])"
   },
   "statue-of-liberty.las"
]
```

Additional attributes created by the
{ref}`EPT addon writer <writers.ept_addon>` can be referenced with the `addon` option.  Here is an example that overrides the `Classification` dimension with an addon dimension derived from the original dataset:

```json
[
    {
        "type": "readers.ept",
        "filename": "https://na-c.entwine.io/autzen/ept.json",
        "addons": { "Classification": "~/entwine/addons/autzen/smrf" }
    },
    {
        "type": "writers.las",
        "filename": "autzen-ept-smrf.las"
    }
]
```

For more details about addon dimensions and how to produce them, see {ref}`writers.ept_addon`.

```{note}
The `forward` option of {ref}`writers.copc` or {ref}`writers.las` cannot work
with EPT due to how EPT can mix content and files. There is no single unified metadata
value to forward. You will have to explicitly set any output options that you
would expect to come from EPT on any writers.
```

```{note}
You can use <viewer.copc.io> to view [EPT], [COPC], and [COG] data directly in
your browser.
```

## Options

filename

: Path to the EPT resource from which to read, ending with `ept.json`.
  Refer to {ref}`filespec` \[Required\]

spatialreference

: Spatial reference to apply to the data.  Overrides any SRS in the input
  itself.  Can be specified as a WKT, proj.4 or EPSG string. \[Default: none\]

bounds

: The extent of the to select in 2 or 3 dimensions. If omitted, the
  entire dataset will be selected.

  ```{include} bounds_opts.md
  ```

resolution

: A point resolution limit to select, expressed as a grid cell edge length.  Units
  correspond to resource coordinate system units.  For example, for a coordinate system
  expressed in meters, a `resolution` value of `0.1` will select points up to a
  ground resolution of 100 points per square meter.

  The resulting resolution may not be exactly this value: the minimum possible resolution
  that is at *least* as precise as the requested resolution will be selected.  Therefore
  the result may be a bit more precise than requested.

addons

: A mapping of assignments of the form `DimensionName: AddonPath`, which
  assigns dimensions from the specified paths to the named dimensions.
  These addon dimensions are created by the
  {ref}`EPT addon writer <writers.ept_addon>`.  If the dimension names
  already exist in the EPT [Schema] for the given resource, then their
  values will be overwritten with those from the appropriate addon.

  Addons may used to override well-known {ref}`dimension <dimensions>`.  For example,
  an addon assignment of `"Classification": "~/addons/autzen/MyGroundDimension/"`
  will override an existing EPT `Classification` dimension with the custom dimension.

origin

: EPT datasets are lossless aggregations of potentially multiple source
  files.  The *origin* option can be used to select all points from a
  single source file.  This option may be specified as a string or an
  integral ID.

  The string form of this option selects a source file by its original
  file path.  This may be a substring instead of the entire path, but
  the string must uniquely select only one source file (via substring
  search).  For example, for an EPT dataset created from source files
  *one.las*, *two.las*, and *two.bpf*, "one" is a sufficient selector,
  but "two" is not.

  The integral form of this option selects a source file by its `OriginId`
  dimension, which can be determined from  the file's position in EPT
  metadata file `entwine-files.json`.

```{note}
When using `pdal info --summary`, using the `origin` option will cause the
resulting bounds to be clipped to those of the selected origin, and the resulting
number of points to be an upper bound for this selection.
```

polygon

: The clipping polygon, expressed in a well-known text string,
  e.g.: `POLYGON((0 0, 5000 10000, 10000 0, 0 0))`.  This option can be
  specified more than once by placing values in an array, in which case all of
  them will be unioned together, acting as a single multipolygon. The polygon definition
  can be followed by a slash ('/') and a spatial reference specification to apply to
  the polygon.

```{note}
When using `pdal info --summary`, using the `polygon` option will cause the
resulting bounds to be clipped to the maximal extents of all provided polygons,
and the resulting number of points to be an upper bound for this polygon selection.
```

```{note}
When both the `bounds` and `polygon` options are specified, only
the points that fall within *both* the bounds and the polygon(s) will be
returned.
```

ogr

: A JSON object representing an OGR query to fetch polygons to use for filtering. The polygons
  fetched from the query are treated exactly like those specified in the `polygon` option.
  The JSON object is specified as follows:

```{include} ogr_json.md
```

requests

: Maximum number of simultaneous requests for EPT data. \[Minimum: 4\] \[Default: 15\]

ignore_unreadable

: If set to true, ignore errors for missing or unreadable point data nodes.

[entwine]: https://entwine.io/
[entwine point tile]: https://entwine.io/entwine-point-tile.html
[EPT]: https://entwine.io/entwine-point-tile.html
[COPC]: https://copc.io
[COG]: https://cogeo.org
[potree]: http://potree.entwine.io/data/nyc.html
[schema]: https://entwine.io/entwine-point-tile.html#schema
