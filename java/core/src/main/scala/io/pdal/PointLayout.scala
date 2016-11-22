package io.pdal

import java.util
import scala.collection.JavaConversions._

class PointLayout extends Native {
  def dimSize(dimType: DimType): Long = dimSize(dimType.id)
  def dimPackedOffset(dimType: DimType): Long = dimPackedOffset(dimType.id)

  def sizedDimTypes(): util.Map[String, SizedDimType] = toSizedDimTypes(dimTypes())
  def toSizedDimTypes(dimTypes: Array[DimType]): util.Map[String, SizedDimType] = {
    var (i, offset, length) = (0, 0l, dimTypes.length)
    val result = new util.HashMap[String, SizedDimType]()
    while(i < length) {
      val dt = dimTypes(i)
      val size = dimSize(dt)
      result += dt.id -> SizedDimType(dt, size, offset)
      offset += size
      i += 1
    }
    result
  }

  @native def dimTypes(): Array[DimType]
  @native def findDimType(name: String): DimType
  @native def dimSize(id: String): Long
  /**
    * Offset of a dim in a packed points byte array calculated as a sum of previous dim sizes.
    * Valid for a point with all dims.
    */
  @native def dimPackedOffset(id: String): Long
  @native def pointSize(): Long
  @native def dispose(): Unit
}
