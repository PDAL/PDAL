package io.pdal

import java.nio.ByteBuffer
import java.util

import scala.collection.JavaConversions._

/**
  * PackedPoints abstraction to work with packed point(s) in JVM memory.
  * SizedDimType contains size and offset for a particular packed point with the current set of dims.
  **/
case class PackedPoints(bytes: Array[Byte],
                        dimTypes: util.Map[String, SizedDimType],
                        metadata: String = "",
                        schema: String = "",
                        proj4String: String = "",
                        WKTString: String = "")
{
  def pointSize: Long = dimTypes.values.map(_.size).sum
  def length: Int = (bytes.length / pointSize).toInt
  def isPoint: Boolean = length == pointSize
  def dimSize(dim: SizedDimType) = dimTypes(dim.dimType.id).size
  def dimSize(dim: DimType) = dimTypes(dim.id).size
  def dimSize(dim: String) = dimTypes(dim).size
  def findDimType(dim: String) = dimTypes(dim).dimType
  def findSizedDimType(dim: String) = dimTypes(dim)

  /**
    * Reads a packed point by point id from a set of packed points.
    */
  def get(i: Int): Array[Byte] = {
    val pointSize = this.pointSize
    if (isPoint) bytes
    else {
      val from = (i * pointSize).toInt
      val to = {
        val t = (from + pointSize).toInt
        if (t > length) length else t
      }

      bytes.slice(from, to)
    }
  }

  def get(idx: Int, dim: SizedDimType): ByteBuffer = get(idx, dim.dimType.id)
  def get(idx: Int, dim: DimType): ByteBuffer = get(idx, dim.id)
  def get(idx: Int, dim: String): ByteBuffer = ByteBuffer.wrap(get(get(idx), dim))

  def get(idx: Int, dims: Array[SizedDimType]): ByteBuffer = ByteBuffer.wrap(get(get(idx), dims.map(_.dimType.id)))
  def get(idx: Int, dims: Array[DimType]): ByteBuffer = ByteBuffer.wrap(get(get(idx), dims.map(_.id)))
  def get(idx: Int, dims: Array[String]): ByteBuffer = ByteBuffer.wrap(get(get(idx), dims))

  def getX(idx: Int): Double = get(idx, DimType.Id.X).getDouble
  def getY(idx: Int): Double = get(idx, DimType.Id.Y).getDouble
  def getZ(idx: Int): Double = get(idx, DimType.Id.Z).getDouble

  /**
    * Reads dim from a packed point.
    */
  def get(packedPoint: Array[Byte], dim: String): Array[Byte] = {
    val sdt = dimTypes(dim)
    val from = sdt.offset.toInt
    val to = from + sdt.size.toInt
    packedPoint.slice(from, to)
  }

  /**
    * Reads dims from a packed point.
    */
  private def get(packedPoint: Array[Byte], dims: Array[String]): Array[Byte] =
    dims.map(get(bytes, _)).fold(Array[Byte]())(_ ++ _)
}
