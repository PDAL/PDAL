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
    * Reads dim from a packed point
    */
  def get(packedPoint: Array[Byte])(dim: DimType): ByteBuffer =
    get(packedPoint, ByteOrder.LITTLE_ENDIAN)(dim)

  def get(packedPoint: Array[Byte], order: ByteOrder)(dim: DimType): ByteBuffer = {
    val from = layout.dimOffset(dim).toInt
    val to = from + layout.dimSize(dim).toInt
    ByteBuffer.wrap(packedPoint.slice(from, to)).order(order)
  }
  /**
    * One dimension read; for multiple dims custom logic required.
    * By default data read in a little endian order
    */
  def get(dim: DimType, i: Int): ByteBuffer = get(dim, i, ByteOrder.LITTLE_ENDIAN)
  def get(dim: String, i: Int): ByteBuffer = get(findDimType(dim), i, ByteOrder.LITTLE_ENDIAN)
  def get(dim: DimType, i: Int, order: ByteOrder): ByteBuffer =
    ByteBuffer.wrap(getPackedPoint(Array(dim), i)).order(order)

  def getX(i: Int): Double = get(DimType.X, i).getDouble
  def getY(i: Int): Double = get(DimType.Y, i).getDouble
  def getZ(i: Int): Double = get(DimType.Z, i).getDouble

  def getX(packedPoint: Array[Byte]): Double = get(packedPoint)(DimType.X).getDouble
  def getY(packedPoint: Array[Byte]): Double = get(packedPoint)(DimType.Y).getDouble
  def getZ(packedPoint: Array[Byte]): Double = get(packedPoint)(DimType.Z).getDouble

  @native def layout: PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getCrsProj4: String
  @native def getCrsWKT(mode_flag: Int, pretty: Boolean): String
  @native def getPackedPoint(dims: Array[DimType], idx: Long): Array[Byte]
  @native def getPackedPoints(dims: Array[DimType]): Array[Byte]
  @native def dispose(): Unit
}
