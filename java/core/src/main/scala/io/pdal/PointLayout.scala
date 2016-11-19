package io.pdal

class PointLayout extends Native {
  def dimSize(dimType: DimType): Long = dimSize(dimType.id)
  def dimPackedOffset(dimType: DimType): Long = dimPackedOffset(dimType.id)

  @native def dimTypes(): Array[DimType]
  @native def findDimType(name: String): DimType
  @native def dimSize(id: String): Long
  @native def dimPackedOffset(id: String): Long
  @native def pointSize(): Long
  @native def dispose(): Unit
}
