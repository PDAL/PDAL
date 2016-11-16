package io.pdal

/** Contains string point type and string point id */
case class DimType(id: String, `type`: String, scale: Double = 1, offset: Double = 0)

object DimType {
  def X = DimType("X", "double")
  def Y = DimType("Y", "double")
  def Z = DimType("Z", "double")
}
