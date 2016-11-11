package com.azavea.pdal

class PointViewIterator extends Iterator[PointView] with Native {
  @native def layout: PointLayout
  @native def hasNext: Boolean
  @native def next(): PointView
}
