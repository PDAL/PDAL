package io.pdal

class PointViewIterator extends Iterator[PointView] with Native {
  @native def hasNext: Boolean
  @native def next(): PointView
  @native def dispose(): Unit
}
