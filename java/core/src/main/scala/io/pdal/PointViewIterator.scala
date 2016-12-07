package io.pdal

import java.util

class PointViewIterator extends util.Iterator[PointView] with Native {
  @native def hasNext: Boolean
  @native def next(): PointView
  @native def dispose(): Unit
}
