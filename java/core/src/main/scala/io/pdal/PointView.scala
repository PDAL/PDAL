package io.pdal

import java.nio.ByteBuffer
import java.util

class PointView extends Native {
  def getPointCloud(idx: Long): PointCloud = getPointCloud(idx, layout.dimTypes())
  def getPointCloud(idx: Long, dims: Array[DimType]): PointCloud =
    PointCloud(
      bytes       = getPackedPoint(idx, dims),
      dimTypes    = layout.toSizedDimTypes(dims)
    )

  def getPointCloud(): PointCloud = getPointCloud(layout.dimTypes())
  def getPointCloud(dims: Array[DimType]): PointCloud =
    PointCloud(
      bytes       = getPackedPoints(dims),
      dimTypes    = layout.toSizedDimTypes(dims)
    )

  def getPackedPoint(idx: Long): Array[Byte] = getPackedPoint(idx, layout.dimTypes())
  def getPackedPoints(): Array[Byte] = getPackedPoints(layout.dimTypes())
  def findDimType(name: String): DimType = layout.findDimType(name)
  def length(): Int = size()
  def getCrsWKT(mode_flag: Int): String = getCrsWKT(mode_flag, pretty = false)
  def getCrsWKT(): String = getCrsWKT(1, pretty = false)

  /**
    * Reads a packed point by point id from a set of packed points.
    */
  def get(idx: Int, packedPoints: Array[Byte]): Array[Byte] = get(idx, packedPoints, layout.dimTypes())
  def get(idx: Int, packedPoints: Array[Byte], dims: Array[DimType]): Array[Byte] = {
    val pointSize = dims.map(layout.dimSize(_)).sum.toInt
    val from = idx * pointSize
    val to = from + pointSize

    util.Arrays.copyOfRange(packedPoints, from, to)
  }

  /**
    * Reads dim from a packed point, point should contain all layout dims.
    */
  def get(packedPoint: Array[Byte], dim: DimType): ByteBuffer = {
    val from = layout.dimPackedOffset(dim).toInt
    val to = from + layout.dimSize(dim).toInt

    ByteBuffer.wrap(util.Arrays.copyOfRange(packedPoint, from, to))
  }
  /**
    * One dimension read; for multiple dims custom logic required.
    */

  def getDouble(idx: Int, dim: String): Double = getDouble(idx, findDimType(dim))
  def getDouble(idx: Int, dim: DimType): Double = get(idx, dim).getDouble

  def getFloat(idx: Int, dim: String): Float = getFloat(idx, findDimType(dim))
  def getFloat(idx: Int, dim: DimType): Float = get(idx, dim).getFloat

  def getLong(idx: Int, dim: String): Long = getLong(idx, findDimType(dim))
  def getLong(idx: Int, dim: DimType): Long = get(idx, dim).getLong

  def getInt(idx: Int, dim: String): Int = getInt(idx, findDimType(dim))
  def getInt(idx: Int, dim: DimType): Int = get(idx, dim).getInt

  def getShort(idx: Int, dim: String): Short = getShort(idx, findDimType(dim))
  def getShort(idx: Int, dim: DimType): Short = get(idx, dim).getShort

  def getChar(idx: Int, dim: String): Char = getChar(idx, findDimType(dim))
  def getChar(idx: Int, dim: DimType): Char = get(idx, dim).getChar

  def getByte(idx: Int, dim: String): Byte = getByte(idx, findDimType(dim))
  def getByte(idx: Int, dim: DimType): Byte = get(idx, dim).get()

  def get(idx: Int, dim: String): ByteBuffer = get(idx, findDimType(dim))
  def get(idx: Int, dim: DimType): ByteBuffer =
    ByteBuffer.wrap(getPackedPoint(idx, Array(dim)))

  def getX(idx: Int): Double = get(idx, DimType.X).getDouble
  def getY(idx: Int): Double = get(idx, DimType.Y).getDouble
  def getZ(idx: Int): Double = get(idx, DimType.Z).getDouble

  def getX(packedPoint: Array[Byte]): Double = get(packedPoint, DimType.X).getDouble
  def getY(packedPoint: Array[Byte]): Double = get(packedPoint, DimType.Y).getDouble
  def getZ(packedPoint: Array[Byte]): Double = get(packedPoint, DimType.Z).getDouble

  @native def layout(): PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getCrsProj4(): String
  @native def getCrsWKT(mode_flag: Int, pretty: Boolean): String
  @native def getPackedPoint(idx: Long, dims: Array[DimType]): Array[Byte]
  @native def getPackedPoints(dims: Array[DimType]): Array[Byte]
  @native def dispose(): Unit
}
