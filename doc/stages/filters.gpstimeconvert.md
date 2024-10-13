(filters.gpstimeconvert)=

# filters.gpstimeconvert

The **gpstimeconvert** filter converts between three GPS time standards found in
lidar data:

1. GPS time (gt)
2. GPS standard time (gst), also known as GPS adjusted time
3. GPS week seconds (gws)

Since GPS week seconds are ambiguous (they reset to 0 at the start of each new
GPS week), care must be taken when they are the source or destination of a
conversion:

- When converting from GPS week seconds, the GPS week number must be known. This
  is accomplished by specifying the [start_date] (in the GMT time zone) on which
  the data collection started. The filter will resolve the ambiguity using the
  supplied start date.
- When converting from GPS week seconds and the times span a new GPS week, the
  presence or absence of week second wrapping must be specified with the
  [wrapped] option. Wrapped week seconds reset to 0 at the start of a new week;
  unwrapped week seconds are allowed to exceed 604800 (60x60x24x7) seconds.
- When converting to GPS week seconds, the week second wrapping preference
  should be specified with the [wrap] option.

```{note}
The filter assumes points are ordered by ascending time, which can be
accomplished by running {ref}`filters.sort` prior to
`filters.gpstimeconvert`. Note that GPS week second times that span a new
GPS week should not be sorted unless they are unwrapped.
```

## Example #1

Convert from GPS time to GPS standard time.

```json
[
    "input.las",
    {
        "type":"filters.gpstimeconvert",
        "conversion":"gt2gst"
    },
    "output.las"
]
```

## Example #2

Convert from GPS standard time to unwrapped GPS week seconds.

```json
[
    "input.las",
    {
        "type":"filters.sort",
        "dimension":"GpsTime",
        "order":"ASC"
    },
    {
        "type":"filters.gpstimeconvert",
        "conversion":"gst2gws",
        "wrap":false
    }
]
```

## Example #3

Convert from wrapped GPS week seconds to GPS time.

```json
[
    "input.las",
    {
        "type":"filters.gpstimeconvert",
        "conversion":"gws2gt",
        "start_date":"2020-12-12",
        "wrapped":true
    },
    "output.las"
]
```

## Options

conversion

: The time conversion. Must be one of the following: "gst2gt", "gst2gws",
  "gt2gst", "gt2gws", "gws2gst", or "gws2gt". \[Required\]

start_date

: When the input times are in GPS week seconds, the date on which the data
  collection started must be supplied in the GMT time zone. Must be in
  "YYYY-MM-DD" format. \[Required for the "gws2gt" and "gws2gst" conversions\]

wrap

: Whether to output wrapped (true) or unwrapped (false) GPS week seconds.
  \[Default: false\]

wrapped

: Specifies whether input GPS week seconds are wrapped (true) or unwrapped
  (false). \[Default: false\]
