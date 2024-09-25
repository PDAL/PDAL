(filters.h3)=

# filters.h3

The **H3 filter** adds a [H3](https://h3geo.org/docs/api/indexing/) ID at a given `resolution`. The
`uint64_t` integer corresponds to the [H3 index](https://h3geo.org/docs/core-library/latLngToCellDesc) of the point.

```{eval-rst}
.. streamable::
```

```{warning}
{ref}`filters.h3` internally depends on being able to reproject the coordinate system to `EPSG:4326`.
If the data does not have coordinate system information, the filter will throw an error.
```

## Options

resolution

: The H3 resolution \[Default: 0\]
