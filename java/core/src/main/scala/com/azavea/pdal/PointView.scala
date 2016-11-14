package com.azavea.pdal

class PointView extends Native {
  @native def layout: PointLayout
  @native def size(): Int
  @native def empty(): Boolean

  // This grabs the points into some
  // class that contains members which represent
  // the data contained in the layout.
  @native def points[T]: Iterable[T]
  @native def dispose(): Unit
  @native def test(): Unit
}
