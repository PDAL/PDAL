package io.pdal

import java.util
import scala.collection.JavaConversions._

class PointLayout extends Native {
  def dimSize(dimType: DimType): Long = dimSize(dimType.id)
  def dimPackedOffset(dimType: DimType): Long = dimPackedOffset(dimType.id)

  def sizedDimTypes(): util.Map[String, SizedDimType] = toSizedDimTypes(dimTypes())
  def toSizedDimTypes(dimTypes: Array[DimType]): util.Map[String, SizedDimType] = {
    dimTypes.map { dt =>
      dt.id -> SizedDimType(dt, dimSize(dt), dimPackedOffset(dt))
    }.toMap[String, SizedDimType]
  }

  @native def dimTypes(): Array[DimType]
  @native def findDimType(name: String): DimType
  @native def dimSize(id: String): Long
  @native def dimPackedOffset(id: String): Long
  @native def pointSize(): Long
  @native def dispose(): Unit
}
