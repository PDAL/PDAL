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

import scala.collection.JavaConversions._

class PointCloudSpec extends TestEnvironmentSpec {
  var packedPoints: PointCloud = _

  describe("PointCloud in JVM memory operations") {
    it("should init PackedPoints") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()

      packedPoints = pv.getPointCloud

      pv.dispose()
      pvi.dispose()
    }

    it("should have a valid point view size") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      pv.length should be (packedPoints.length)
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid (X, Y, Z) data") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      pv.getX(0) should be (packedPoints.getX(0))
      pv.getY(0) should be (packedPoints.getY(0))
      pv.getZ(0) should be (packedPoints.getZ(0))
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid packed data") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      val layout = pv.layout
      val arr = pv.getPackedPoint(0, Array(DimType.X, DimType.Y))
      val (xarr, yarr) = arr.take(layout.dimSize(DimType.X).toInt) -> arr.drop(layout.dimSize(DimType.Y).toInt)

      val marr = packedPoints.get(0, Array(DimType.X, DimType.Y))
      val (xmarr, ymarr) = arr.take(packedPoints.dimSize(DimType.X).toInt) -> arr.drop(packedPoints.dimSize(DimType.Y).toInt)

      xarr should be (xmarr)
      yarr should be (ymarr)
      ByteBuffer.wrap(xmarr).order(ByteOrder.nativeOrder()).getDouble should be (pv.getX(0))
      ByteBuffer.wrap(ymarr).order(ByteOrder.nativeOrder()).getDouble should be (pv.getY(0))

      layout.dispose()
      pv.dispose()
      pvi.dispose()
    }

    it("should read the whole packed point and grab only one dim") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      packedPoints.get(0, DimType.Y).getDouble should be (pv.getY(0))
      pv.dispose()
      pvi.dispose()
    }

    it("should read all packed points and grab only one point out of it") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      pv.get(3, pv.getPackedPoints) should be (packedPoints.get(3))
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid value by name") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      pv.getByte(0, "ReturnNumber") should be (packedPoints.getByte(0, "ReturnNumber"))
      pv.dispose()
      pvi.dispose()
    }

    it("should read correctly data as a packed point") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      packedPoints.dimTypes.foreach { case (_, sdt) =>
        pv.get(0, sdt.dimType) should be (packedPoints.get(0, sdt))
      }
      pv.dispose()
      pvi.dispose()
    }

    it("layout should have a valid number of dims") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      pv.layout.dimTypes().length should be (packedPoints.dimTypes.size)
      pv.dispose()
      pvi.dispose()
    }

    it("should find a dim by name") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      pv.findDimType("Red") should be (packedPoints.findDimType("Red"))
      pv.dispose()
      pvi.dispose()
    }

    it("dim sizes should be of a valid size") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      val layout = pv.layout
      layout.dimTypes().map(pv.layout.dimSize(_)).sum should be (packedPoints.pointSize)
      layout.dispose()
      pv.dispose()
      pvi.dispose()
    }

    it("should read all packed points valid") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      val length = packedPoints.bytes.length
      pv.getPackedPoints.length should be (length)
      pv.dispose()
      pvi.dispose()
    }

    it("should get correct points and all values") {
      val pvi = pipeline.getPointViews()
      val pv = pvi.next()
      val length = pv.length
      val dimTypes = packedPoints.dimTypes.values().map(_.dimType)
      for (i <- 0 until length) {
        packedPoints.get(i) should be (pv.getPackedPoint(i))
        packedPoints.getX(i) should be (pv.getX(i))
        packedPoints.getY(i) should be (pv.getY(i))
        packedPoints.getZ(i) should be (pv.getZ(i))
        dimTypes.foreach { dt =>
          packedPoints.get(i, dt).array() should be (pv.get(i, dt).array())
        }
      }
      pv.dispose()
      pvi.dispose()
    }
  }

  override def beforeAll() = {
    pipeline.execute()
  }
}
