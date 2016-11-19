package io.pdal

import scala.collection.JavaConversions._
import java.nio.ByteBuffer

class PackedPointsSpec extends TestEnvironmentSpec {
  var packedPoints: PackedPoints = _

  describe("PackedPoints in JVM memory operations") {
    it("should init PackedPoints") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()

      packedPoints = pv.getPackedPointsWithMetadata(
        metadata = pipeline.getMetadata(),
        schema   = pipeline.getSchema()
      )

      pv.dispose()
      pvi.dispose()
    }

    it("should have a valid point view size") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.length should be (packedPoints.length)
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid (X, Y, Z) data") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.getX(0) should be (packedPoints.getX(0))
      pv.getY(0) should be (packedPoints.getY(0))
      pv.getZ(0) should be (packedPoints.getZ(0))
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid packed data") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      val layout = pv.layout
      val arr = pv.getRawPackedPoint(0, Array(DimType.X, DimType.Y))
      val (xarr, yarr) = arr.take(layout.dimSize(DimType.X).toInt) -> arr.drop(layout.dimSize(DimType.Y).toInt)

      val marr = packedPoints.get(0, Array(DimType.X, DimType.Y))
      val (xmarr, ymarr) = arr.take(packedPoints.dimSize(DimType.X).toInt) -> arr.drop(packedPoints.dimSize(DimType.Y).toInt)

      xarr should be (xmarr)
      yarr should be (ymarr)
      ByteBuffer.wrap(xmarr).getDouble should be (pv.getX(0))
      ByteBuffer.wrap(ymarr).getDouble should be (pv.getY(0))

      layout.dispose()
      pv.dispose()
      pvi.dispose()
    }

    it("should read the whole packed point and grab only one dim") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      packedPoints.get(0, DimType.Y).getDouble should be (pv.getY(0))
      pv.dispose()
      pvi.dispose()
    }

    it("should read all packed points and grab only one point out of it") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.get(3, pv.getRawPackedPoints) should be (packedPoints.get(3))
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid value by name") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.get(0, "ReturnNumber").get() & 0xff should be (packedPoints.get(0, "ReturnNumber").get() & 0xff)
      pv.dispose()
      pvi.dispose()
    }

    it("should read correctly data as a packed point") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      packedPoints.dimTypes.foreach { case (_, sdt) =>
        pv.get(0, sdt.dimType) should be (packedPoints.get(0, sdt))
      }
      pv.dispose()
      pvi.dispose()
    }

    it("layout should have a valid number of dims") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.layout.dimTypes().length should be (packedPoints.dimTypes.size)
      pv.dispose()
      pvi.dispose()
    }

    it("should find a dim by name") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.findDimType("Red") should be (packedPoints.findDimType("Red"))
      pv.dispose()
      pvi.dispose()
    }

    it("dim sizes should be of a valid size") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      val layout = pv.layout
      layout.dimTypes().map(pv.layout.dimSize(_)).sum should be (packedPoints.pointSize)
      layout.dispose()
      pv.dispose()
      pvi.dispose()
    }

    it("should read all packed points valid") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      val length = packedPoints.bytes.length
      pv.getRawPackedPoints.length should be (length)
      length should be (packedPoints.pointSize * packedPoints.length)
      pv.dispose()
      pvi.dispose()
    }

    it("should read crs correct") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      packedPoints.proj4String should be (proj4String)
      pv.dispose()
      pvi.dispose()
    }
  }

  override def beforeAll() = {
    pipeline.execute()
  }
}
