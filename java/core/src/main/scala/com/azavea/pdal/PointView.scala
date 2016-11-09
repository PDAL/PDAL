package com.azavea.pdal

trait PointView {
  // This grabs the points into some
  // class that contains members which represent
  // the data contained in the layout.
  @native def points[T]: Iterable[T]
}
