package io.pdal

import scala.collection.JavaConversions._
import java.nio.{ByteBuffer, ByteOrder}

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
