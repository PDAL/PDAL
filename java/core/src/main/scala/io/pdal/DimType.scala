package io.pdal

case class DimType(id: String, `type`: String, scale: Double = 1, offset: Double = 0)

object DimType {
  object Id {
    val Unknown = "Unknown"
    val X = "X"
    val Y = "Y"
    val Z = "Z"
  }

  def X = DimType(Id.X, "double")
  def Y = DimType(Id.Y, "double")
  def Z = DimType(Id.Z, "double")
}
