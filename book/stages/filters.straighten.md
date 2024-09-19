(filters.straighten)=

# filters.straighten

The **straighten filter** transforms the point cloud in a new parametric coordinate system,
each point in world coordinate (X,Y,Z) is being projected along closest poyline segment,
and rotated along the segment accordingly to the average m/roll value.

```{eval-rst}
.. streamable::
```

```{note}
The new coordinate system (X', Y', Z') could be understood as :
\* X' : curvilinear abcissa (or meter point)
\* Y' : orthogonal distance to segment (or orthogonal distance to line)
\* Z' : orthogonal distance from (rolling) plane
```

## Examples

```json
[
    "input.las",
    {
        "type": "filters.straighten",
        "polyline" : "LINSTRING ZM (...)"
    },
    "straighten.las"
]
```

```json
[
    "input.las",
    {
        "type": "filters.straighten",
        "polyline" : "LINSTRING ZM (...)"
    },
    "straighten.las"
]
```

## Options

polyline

: `` wkt` `` or `` json` `` definition of a 3D linestring with measurment (LINESTRING ZM in wkt) along which the cloud will be straighten.
  M is supposed to be roll expressed in radians. This is mandatory.

offset

: if you want to add an X' during straightening operation (or take an offset into account while unstraightening).
  This can be understood as a starting meter point. \[Default: 0\]

reverse

: whether to straighten or unstraighten the point cloud \[Default: `false`\]

```{eval-rst}
.. include:: filter_opts.rst
```
