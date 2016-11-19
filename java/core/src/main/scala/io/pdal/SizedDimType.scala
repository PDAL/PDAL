package io.pdal

case class SizedDimType(dimType: DimType, size: Long, offset: Long)

object SizedDimType {
  def asMap(layout: PointLayout): Map[String, SizedDimType] = {
    layout.dimTypes().map { dt =>
      dt.id -> SizedDimType(dt, layout.dimSize(dt), layout.dimPackedOffset(dt))
    } toMap
  }
}
