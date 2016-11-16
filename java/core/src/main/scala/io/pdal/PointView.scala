package io.pdal

class PointView extends Native {
  def getPackedPoint(idx: Long): Array[Byte] = getPackedPoint(layout.dimTypes(), idx)
  def getPackedPoints: Array[Byte] = getPackedPoints(layout.dimTypes())

  @native def layout: PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getPackedPoint(dims: Array[DimType], idx: Long): Array[Byte]
  @native def getPackedPoints(dims: Array[DimType]): Array[Byte]
  @native def dispose(): Unit
}
