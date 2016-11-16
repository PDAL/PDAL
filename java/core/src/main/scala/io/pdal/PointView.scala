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
    * One dimension read; for multiple dims custom logic required.
    * By default data read in a little endian order
    */
  def get(dim: DimType, i: Int): ByteBuffer = get(dim, i, ByteOrder.LITTLE_ENDIAN)
  def get(dim: String, i: Int): ByteBuffer = get(findDimType(dim), i, ByteOrder.LITTLE_ENDIAN)
  def get(dim: DimType, i: Int, order: ByteOrder): ByteBuffer =
    ByteBuffer.wrap(getPackedPoint(Array(dim), i)).order(order)

  def getX(i: Int) = get(DimType.X, i).getDouble
  def getY(i: Int) = get(DimType.Y, i).getDouble
  def getZ(i: Int) = get(DimType.Z, i).getDouble

  @native def layout: PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getCrsProj4: String
  @native def getCrsWKT(mode_flag: Int, pretty: Boolean): String
  @native def getPackedPoint(dims: Array[DimType], idx: Long): Array[Byte]
  @native def getPackedPoints(dims: Array[DimType]): Array[Byte]
  @native def dispose(): Unit
}
