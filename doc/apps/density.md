(density_command)=

# density

```{warning}
As of PDAL v2.6.0, the `density` command is marked as DEPRECATED. It will
be removed from the default install in PDAL v2.7 and removed completely in
PDAL v2.8.
```

The density command produces a tessellated hexagonal [OGR layer] from the
output of {ref}`filters.hexbin`.

```
$ pdal density <input> <output>
```

```
--input, -i        Input point cloud file name
--output, -o       Output vector data source
--lyr_name         OGR layer name to write into datasource
--ogrdriver, -f    OGR driver name to use
--sample_size      Sample size for automatic edge length calculation. [5000]
--threshold        Required cell density [15]
--hole_cull_tolerance_area
                   Tolerance area to apply to holes before cull
--smooth           Smooth boundary output
```

[ogr layer]: https://gdal.org/en/latest/drivers/vector/index.html
