where

: An {ref}`expression <pdal_expression>` that limits points passed to a filter.
  Points that don't pass the
  expression skip the stage but are available to subsequent stages in a pipeline.
  \[Default: no filtering\]

where_merge

: A strategy for merging points skipped by a `where` option when running in standard mode.
  If `true`, the skipped points are added to the first point view returned by the skipped
  filter. If `false`, skipped points are placed in their own point view. If `auto`,
  skipped points are merged into the returned point view provided that only one point view
  is returned and it has the same point count as it did when the filter was run.
  \[Default: `auto`\]
