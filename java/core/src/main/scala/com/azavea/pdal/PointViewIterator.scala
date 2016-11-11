package com.azavea.pdal

class PointViewIterator extends Iterator[PointView] with Native {
  @native def layout: PointLayout

  override def hasNext: Boolean = ???

  override def next(): PointView = ???
}
