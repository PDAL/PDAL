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

import java.util
import scala.collection.JavaConversions._

class PointLayout extends Native {
  def dimSize(dimType: DimType): Long = dimSize(dimType.id)
  def dimPackedOffset(dimType: DimType): Long = dimPackedOffset(dimType.id)

  def sizedDimTypes(): util.Map[String, SizedDimType] = toSizedDimTypes(dimTypes())
  def toSizedDimTypes(dimTypes: Array[DimType]): util.Map[String, SizedDimType] = {
    var (i, offset, length) = (0, 0l, dimTypes.length)
    val result = new util.HashMap[String, SizedDimType]()
    while(i < length) {
      val dt = dimTypes(i)
      val size = dimSize(dt)
      result += dt.id -> SizedDimType(dt, size, offset)
      offset += size
      i += 1
    }
    result
  }

  @native def dimTypes(): Array[DimType]
  @native def findDimType(name: String): DimType
  @native def dimSize(id: String): Long
  /**
    * Offset of a dim in a packed points byte array calculated as a sum of previous dim sizes.
    * Valid for a point with all dims.
    */
  @native def dimPackedOffset(id: String): Long
  @native def pointSize(): Long
  @native def dispose(): Unit
}
