package io.pdal

import java.nio.{ByteBuffer, ByteOrder}
class PointView extends Native {
  def getPackedPoint(idx: Long): Array[Byte] = getPackedPoint(layout.dimTypes(), idx)
  def getPackedPoints: Array[Byte] = getPackedPoints(layout.dimTypes())
  def findDimType(name: String): DimType = layout.findDimType(name)
  def length: Int = size()

  def get(dim: DimType, i: Int): ByteBuffer = get(dim, i, ByteOrder.LITTLE_ENDIAN)
  def get(dim: DimType, i: Int, order: ByteOrder): ByteBuffer = ByteBuffer.wrap(getPackedPoint(Array(dim), i)).order(order)

  /*def getXY(i: Int) = {
    val x = findDimType("X")
    val y = findDimType("Y")
    val xy = Array(x, y)
    val arr = getPackedPoint(xy, i)
    println(new String(arr.map(_.toChar)))

    val xarr = arr.take(8)
    val yarr = arr.drop(8)

    println(s"xarr: ${ByteBuffer.wrap(xarr).order(ByteOrder.LITTLE_ENDIAN).getDouble}")
    println(s"yarr: ${ByteBuffer.wrap(yarr).order(ByteOrder.LITTLE_ENDIAN).getDouble}")

    get(Array(x, y), i, ByteOrder.LITTLE_ENDIAN)
  }
*/
  def getX(i: Int) = get(DimType.X, i).getDouble
  def getY(i: Int) = get(DimType.Y, i).getDouble
  def getZ(i: Int) = get(DimType.Z, i).getDouble

  @native def layout: PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getPackedPoint(dims: Array[DimType], idx: Long): Array[Byte]
  @native def getPackedPoints(dims: Array[DimType]): Array[Byte]
  @native def dispose(): Unit
}
