package com.azavea.pdal

class PointView extends Native {
  def getPackedPoint(idx: Long): Array[Byte] = getPackedPoint(layout.dimTypes(), idx)

  @native def layout: PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getPackedPoint(dims: java.util.List[DimType], idx: Long): Array[Byte]
  @native def getPackedPoints(dims: java.util.List[DimType]): Array[Byte]
  @native def dispose(): Unit
  @native def test(): Unit
}
