package io.pdal

import java.nio.{ByteBuffer, ByteOrder}

class PointView extends Native {
  def getPackedPoint(idx: Long): Array[Byte] = getPackedPoint(layout.dimTypes(), idx)
  def getPackedPoints: Array[Byte] = getPackedPoints(layout.dimTypes())
  def findDimType(name: String): DimType = layout.findDimType(name)
  def length: Int = size()
  def getCrsWKT(mode_flag: Int): String = getCrsWKT(mode_flag, pretty = false)
  def getCrsWKT: String = getCrsWKT(1, pretty = false)

  /**
    * Reads a packed point by point id from a set of packed points.
    */
  def get(packedPoints: Array[Byte], i: Int): Array[Byte] = get(packedPoints, layout.dimTypes(), i)
  def get(packedPoints: Array[Byte], dims: Array[DimType], i: Int): Array[Byte] = {
    val pointSize = dims.map(layout.dimSize(_)).sum
    val from = (i * pointSize).toInt
    val to = {
      val t = (from + pointSize).toInt
      if(t > length) length else t
    }

    packedPoints.slice(from, to)
  }

  /**
    * Reads dim from a packed point
    */
  def get(packedPoint: Array[Byte], dim: DimType): ByteBuffer = {
    val from = layout.dimOffset(dim).toInt
    val to = from + layout.dimSize(dim).toInt
    ByteBuffer.wrap(packedPoint.slice(from, to)).order(ByteOrder.nativeOrder())
  }
  /**
    * One dimension read; for multiple dims custom logic required.
    */
  def get(dim: String, i: Int): ByteBuffer = get(findDimType(dim), i)
  def get(dim: DimType, i: Int): ByteBuffer =
    ByteBuffer.wrap(getPackedPoint(Array(dim), i)).order(ByteOrder.nativeOrder())

  def getX(i: Int): Double = get(DimType.X, i).getDouble
  def getY(i: Int): Double = get(DimType.Y, i).getDouble
  def getZ(i: Int): Double = get(DimType.Z, i).getDouble

  def getX(packedPoint: Array[Byte]): Double = get(packedPoint, DimType.X).getDouble
  def getY(packedPoint: Array[Byte]): Double = get(packedPoint, DimType.Y).getDouble
  def getZ(packedPoint: Array[Byte]): Double = get(packedPoint, DimType.Z).getDouble

  @native def layout: PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getCrsProj4: String
  @native def getCrsWKT(mode_flag: Int, pretty: Boolean): String
  @native def getPackedPoint(dims: Array[DimType], idx: Long): Array[Byte]
  @native def getPackedPoints(dims: Array[DimType]): Array[Byte]
  @native def dispose(): Unit
}
