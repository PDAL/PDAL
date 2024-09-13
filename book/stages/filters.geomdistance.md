(filters-geomdistance)=

# filters.geomdistance

The geomdistance filter computes the distance between a given polygon
and points.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::

```

## Example 1

This example computes the 2D distance of points to the given geometry.

```json
[
    "autzen.las",
    {
        "type":"filters.geomdistance",
        "geometry":"POLYGON ((636889.412951239268295 851528.512293258565478 422.7001953125,636899.14233423944097 851475.000686757150106 422.4697265625,636899.14233423944097 851475.000686757150106 422.4697265625,636928.33048324030824 851494.459452757611871 422.5400390625,636928.33048324030824 851494.459452757611871 422.5400390625,636928.33048324030824 851494.459452757611871 422.5400390625,636976.977398241520859 851513.918218758190051 424.150390625,636976.977398241520859 851513.918218758190051 424.150390625,637069.406536744092591 851475.000686757150106 438.7099609375,637132.647526245797053 851445.812537756282836 425.9501953125,637132.647526245797053 851445.812537756282836 425.9501953125,637336.964569251285866 851411.759697255445644 425.8203125,637336.964569251285866 851411.759697255445644 425.8203125,637473.175931254867464 851158.795739248627797 435.6298828125,637589.928527257987298 850711.244121236610226 420.509765625,637244.535430748714134 850511.791769731207751 420.7998046875,636758.066280735656619 850667.461897735483944 434.609375,636539.155163229792379 851056.63721774588339 422.6396484375,636889.412951239268295 851528.512293258565478 422.7001953125))",
    },
    "dimension":"distance",
    {
        "type":"writers.las",
        "filename":"with-distance.las"
    }
]
```

:::{figure} ../images/filters.geomdistance-normal-mode.png
:alt: Normal mode distance of Autzen to selection
:scale: 75%

Normal distance mode causes any points *within* the given polygon to have a distance of 0.
:::

:::{figure} ../images/filters.geomdistance-ring-mode.png
:alt: Ring mode distance of Autzen to selection
:scale: 75%

`ring` of `True` causes the polygon external ring to be used
for distance computation, resulting in distances **inside** the
polygon to be computed.
:::

## Options

geometry

: The polygon, expressed in a well-known text string,
  eg: `"POLYGON((0 0, 5000 10000, 10000 0, 0 0))"`.

dimension

: The dimension to write the distance into
  bounds or polygon. \[Default: distance\]

ogr

: An `ogr` block (described in {ref}`readers.ept`)

ring

: Use the outer ring of the polygon (so as to get distances to the exterior
  ring instead of all points inside the polygon having distance `0`).
  \[Default: false\]

```{eval-rst}
.. include:: filter_opts.rst
```
