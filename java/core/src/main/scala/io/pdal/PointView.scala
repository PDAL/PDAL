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

class PointView extends Native {
  def getPointCloud(idx: Long): PointCloud = getPointCloud(idx, layout.dimTypes())
  def getPointCloud(idx: Long, dims: Array[DimType]): PointCloud =
    PointCloud(
      bytes    = getPackedPoint(idx, dims),
      dimTypes = layout.toSizedDimTypes(dims)
    )

  def getPointCloud(): PointCloud = getPointCloud(layout.dimTypes())
  def getPointCloud(dims: Array[DimType]): PointCloud =
    PointCloud(
      bytes    = getPackedPoints(dims),
      dimTypes = layout.toSizedDimTypes(dims)
    )

  def getPackedPoint(idx: Long): Array[Byte] = getPackedPoint(idx, layout.dimTypes())
  def getPackedPoints(): Array[Byte] = getPackedPoints(layout.dimTypes())
  def findDimType(name: String): DimType = layout.findDimType(name)
  def length(): Int = size()
  def getCrsWKT(): String = getCrsWKT(pretty = false)

  /**
    * Reads a packed point by point id from a set of packed points.
    */
  def get(idx: Int, packedPoints: Array[Byte]): Array[Byte] = get(idx, packedPoints, layout.dimTypes())
  def get(idx: Int, packedPoints: Array[Byte], dims: Array[DimType]): Array[Byte] = {
    val pointSize = dims.map(layout.dimSize(_)).sum.toInt
    val from = idx * pointSize
    val result = new Array[Byte](pointSize)
    var j = 0
    while(j < pointSize) {
      result(j) = packedPoints(from + j)
      j += 1
    }
    result
  }

  /**
    * Reads dim from a packed point, point should contain all layout dims.
    */
  def get(packedPoint: Array[Byte], dim: DimType): ByteBuffer = {
    val from = layout.dimPackedOffset(dim).toInt
    val dimSize = layout.dimSize(dim).toInt
    val result = new Array[Byte](dimSize)
    var j = 0
    while(j < dimSize) {
      result(j) = packedPoint(from + j)
      j += 1
    }
    ByteBuffer.wrap(result).order(ByteOrder.nativeOrder())
  }

  def getDouble(packedPoint: Array[Byte], dim: String): Double = getDouble(packedPoint, findDimType(dim))
  def getDouble(packedPoint: Array[Byte], dim: DimType): Double = get(packedPoint, dim).getDouble

  def getFloat(packedPoint: Array[Byte], dim: String): Float = getFloat(packedPoint, findDimType(dim))
  def getFloat(packedPoint: Array[Byte], dim: DimType): Float = get(packedPoint, dim).getFloat

  def getLong(packedPoint: Array[Byte], dim: String): Long = getLong(packedPoint, findDimType(dim))
  def getLong(packedPoint: Array[Byte], dim: DimType): Long = get(packedPoint, dim).getLong

  def getInt(packedPoint: Array[Byte], dim: String): Int = getInt(packedPoint, findDimType(dim))
  def getInt(packedPoint: Array[Byte], dim: DimType): Int = get(packedPoint, dim).getInt

  def getShort(packedPoint: Array[Byte], dim: String): Short = getShort(packedPoint, findDimType(dim))
  def getShort(packedPoint: Array[Byte], dim: DimType): Short = get(packedPoint, dim).getShort

  def getChar(packedPoint: Array[Byte], dim: String): Char = getChar(packedPoint, findDimType(dim))
  def getChar(packedPoint: Array[Byte], dim: DimType): Char = get(packedPoint, dim).getChar

  def getByte(packedPoint: Array[Byte], dim: String): Byte = getByte(packedPoint, findDimType(dim))
  def getByte(packedPoint: Array[Byte], dim: DimType): Byte = get(packedPoint, dim).get()

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
    ByteBuffer.wrap(getPackedPoint(idx, Array(dim))).order(ByteOrder.nativeOrder())

  def getX(idx: Int): Double = getDouble(idx, DimType.X)
  def getY(idx: Int): Double = getDouble(idx, DimType.Y)
  def getZ(idx: Int): Double = getDouble(idx, DimType.Z)

  def getX(packedPoint: Array[Byte]): Double = getDouble(packedPoint, DimType.X)
  def getY(packedPoint: Array[Byte]): Double = getDouble(packedPoint, DimType.Y)
  def getZ(packedPoint: Array[Byte]): Double = getDouble(packedPoint, DimType.Z)

  @native def layout(): PointLayout
  @native def size(): Int
  @native def empty(): Boolean
  @native def getCrsProj4(): String
  @native def getCrsWKT(pretty: Boolean): String
  @native def getPackedPoint(idx: Long, dims: Array[DimType]): Array[Byte]
  @native def getPackedPoints(dims: Array[DimType]): Array[Byte]
  @native def dispose(): Unit
}
