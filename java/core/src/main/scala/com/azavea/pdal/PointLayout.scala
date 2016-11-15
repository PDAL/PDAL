package com.azavea.pdal

class PointLayout extends Native {
  def dimSize(dimType: DimType): Long = dimSize(dimType.id)
  def dimOffset(dimType: DimType): Long = dimOffset(dimType.id)

  /** return java.util.List to have better Java compatibility */
  @native def dimTypes(): java.util.List[DimType]
  @native def findDimType(name: String): DimType
  @native def dimSize(id: String): Long
  @native def dimOffset(id: String): Long
  @native def pointSize(): Long
  @native def dispose(): Unit
}
