package com.azavea.pdal

trait PointViewIterator extends Iterator[PointView] {
  @native def layout: PointLayout
}
