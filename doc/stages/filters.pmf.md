(filters.pmf)=

# filters.pmf

The **Progressive Morphological Filter (PMF)** is a method of
segmenting ground and non-ground returns. This filter is an implementation
of the method described in
{cite:p}`zhang2003progressive`.

```{eval-rst}
.. embed::
```

## Example

```json
[
    "input.las",
    {
        "type":"filters.pmf"
    },
    "output.las"
]
```

## Notes

- [slope] controls the height threshold at each iteration. A slope of 1.0
  represents a 1:1 or 45º.
- [initial_distance] is \_intended\_ to be set to account for z noise, so for a
  flat surface if you have an uncertainty of around 15 cm, you set
  [initial_distance] large enough to not exclude these points from the ground.
- For a given iteration, the height threshold is determined by multiplying
  slope by [cell_size] by the difference in window size between the
  current and last iteration, plus the [initial_distance]. This height
  threshold is constant across all cells and is maxed out at the
  [max_distance] value. If the difference in elevation between a point and its
  “opened” value (from the morphological operator) exceeds the height threshold,
  it is treated as non-ground.  So, bigger slope leads to bigger height
  thresholds, and these grow with each iteration (not to exceed the max).  With
  flat terrain, keep this low, the thresholds are small, and stuff is more
  aggressively dumped into non-ground class.  In rugged terrain, open things up
  a little, but then you can start missing buildings, veg, etc.
- Very large [max_window_size] values will result in a lot of potentially
  extra iteration. This parameter can have a strongly negative impact on
  computation performance.
- [exponential] is used to control the rate of growth of morphological window
  sizes toward [max_window_size]. Linear growth preserves gradually changing
  topographic features well, but demands considerable compute time. The default
  behavior is to grow the window sizes exponentially, thus reducing the number
  of iterations.
- This filter will mark all returns deemed to be ground returns with a
  classification value of 2 (per the LAS specification). To extract only these
  returns, users can add a {ref}`range filter<filters.range>` to the pipeline.

```json
{
  "type":"filters.range",
  "limits":"Classification[2:2]"
}
```

```{note}
{cite:p}`zhang2003progressive` describes the consequences and relationships of the parameters
in more detail and is the canonical resource on the topic.
```

## Options

cell_size

: Cell Size. \[Default: 1\]

exponential

: Use exponential growth for window sizes? \[Default: true\]

ignore

: Range of values to ignore. \[Optional\]

initial_distance

: Initial distance. \[Default: 0.15\]

returns

: Comma-separated list of return types into which data should be segmented.
  Valid groups are "last", "first", "intermediate" and "only". \[Default:
  "last, only"\]

max_distance

: Maximum distance. \[Default: 2.5\]

max_window_size

: Maximum window size. \[Default: 33\]

slope

: Slope. \[Default: 1.0\]

```{include} filter_opts.md
```
