/******************************************************************************
  * Copyright (c) 2016, hobu Inc.  (info@hobu.co)
  *
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following
  * conditions are met:
  *
  *     * Redistributions of source code must retain the above copyright
  *       notice, this list of conditions and the following disclaimer.
  *     * Redistributions in binary form must reproduce the above copyright
  *       notice, this list of conditions and the following disclaimer in
  *       the documentation and/or other materials provided
  *       with the distribution.
  *     * Neither the name of Hobu, Inc. nor the names of its
  *       contributors may be used to endorse or promote products derived
  *       from this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
  * OF SUCH DAMAGE.
  ****************************************************************************/

package io.pdal

import java.nio.{ByteBuffer, ByteOrder}
import java.util

import scala.collection.JavaConversions._

/**
  * PointCloud abstraction to work with packed point(s) in JVM memory.
  * SizedDimType contains size and offset for a particular packed point with the current set of dims.
  **/
case class PointCloud(bytes: Array[Byte], dimTypes: util.Map[String, SizedDimType]) {
  val pointSize: Int = dimTypes.values.map(_.size).sum.toInt
  val length: Int = bytes.length / pointSize
  val isPoint: Boolean = length == pointSize

  def dimSize(dim: SizedDimType) = dimTypes(dim.dimType.id).size
  def dimSize(dim: DimType) = dimTypes(dim.id).size
  def dimSize(dim: String) = dimTypes(dim).size
  def findDimType(dim: String) = dimTypes(dim).dimType
  def findSizedDimType(dim: String) = dimTypes(dim)

  /**
    * Reads a packed point by point id from a set of packed points.
    */
  def get(i: Int): Array[Byte] = {
    if (isPoint) bytes
    else {
      val from = i * pointSize
      val result = new Array[Byte](pointSize)
      var j = 0
      while(j < pointSize) {
        result(j) = bytes(from + j)
        j += 1
      }
      result
    }
  }

  def getDouble(idx: Int, dim: SizedDimType): Double = getDouble(idx, dim.dimType)
  def getDouble(idx: Int, dim: DimType): Double = getDouble(idx, dim.id)
  def getDouble(idx: Int, dim: String): Double = get(idx, dim).getDouble

  def getFloat(idx: Int, dim: SizedDimType): Float = getFloat(idx, dim.dimType.id)
  def getFloat(idx: Int, dim: DimType): Float = getFloat(idx, dim.id)
  def getFloat(idx: Int, dim: String): Float = get(idx, dim).getFloat

  def getLong(idx: Int, dim: SizedDimType): Long = getLong(idx, dim.dimType.id)
  def getLong(idx: Int, dim: DimType): Long = getLong(idx, dim.id)
  def getLong(idx: Int, dim: String): Long = get(idx, dim).getLong

  def getInt(idx: Int, dim: SizedDimType): Int = getInt(idx, dim.dimType.id)
  def getInt(idx: Int, dim: DimType): Int = getInt(idx, dim.id)
  def getInt(idx: Int, dim: String): Int = get(idx, dim).getInt

  def getShort(idx: Int, dim: SizedDimType): Short = getShort(idx, dim.dimType.id)
  def getShort(idx: Int, dim: DimType): Short = getShort(idx, dim.id)
  def getShort(idx: Int, dim: String): Short = get(idx, dim).getShort

  def getChar(idx: Int, dim: SizedDimType): Char = getChar(idx, dim.dimType.id)
  def getChar(idx: Int, dim: DimType): Char = getChar(idx, dim.id)
  def getChar(idx: Int, dim: String): Char = get(idx, dim).getChar

  def getByte(idx: Int, dim: SizedDimType): Byte = getByte(idx, dim.dimType.id)
  def getByte(idx: Int, dim: DimType): Byte = getByte(idx, dim.id)
  def getByte(idx: Int, dim: String): Byte = get(idx, dim).get()

  def get(idx: Int, dim: SizedDimType): ByteBuffer = get(idx, dim.dimType.id)
  def get(idx: Int, dim: DimType): ByteBuffer = get(idx, dim.id)
  def get(idx: Int, dim: String): ByteBuffer = ByteBuffer.wrap(get(get(idx), dim)).order(ByteOrder.nativeOrder())

  def get(idx: Int, dims: Array[SizedDimType]): ByteBuffer = get(idx, dims.map(_.dimType.id))
  def get(idx: Int, dims: Array[DimType]): ByteBuffer = get(idx, dims.map(_.id))
  def get(idx: Int, dims: Array[String]): ByteBuffer = ByteBuffer.wrap(get(get(idx), dims)).order(ByteOrder.nativeOrder())

  def getX(idx: Int): Double = getDouble(idx, DimType.Id.X)
  def getY(idx: Int): Double = getDouble(idx, DimType.Id.Y)
  def getZ(idx: Int): Double = getDouble(idx, DimType.Id.Z)

  /**
    * Reads dim from a packed point.
    */
  def get(packedPoint: Array[Byte], dim: String): Array[Byte] = {
    val sdt = dimTypes(dim)
    val from = sdt.offset.toInt
    val dimSize = sdt.size.toInt
    val result = new Array[Byte](dimSize)
    var j = 0
    while(j < dimSize) {
      result(j) = packedPoint(from + j)
      j += 1
    }
    result
  }

  /**
    * Reads dims from a packed point.
    */
  private def get(packedPoint: Array[Byte], dims: Array[String]): Array[Byte] =
    dims.map(get(bytes, _)).fold(Array[Byte]())(_ ++ _)
}
