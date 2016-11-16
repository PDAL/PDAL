package io.pdal

import java.nio.{ByteBuffer, ByteOrder}

import org.scalatest._

class PipelineSpec extends FunSpec with Matchers with BeforeAndAfterAll {
  def getJson(resource: String): String = {
    val stream = getClass.getResourceAsStream(resource)
    val lines = scala.io.Source.fromInputStream(stream).getLines
    val json = lines.mkString(" ")
    stream.close()
    json
  }

  val json = getJson("/las.json")
  val badJson =
    """
      |{
      |  "pipeline": [
      |    "nofile.las",
      |    {
      |        "type": "filters.sort",
      |        "dimension": "X"
      |    }
      |  ]
      |}
     """.stripMargin

  val proj4String = "+proj=lcc +lat_1=43 +lat_2=45.5 +lat_0=41.75 +lon_0=-120.5 +x_0=400000 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs"

  val pipeline: Pipeline = Pipeline(json)

  describe("Pipeline execution") {

    it("should validate as incorrect json (bad json passed)") {
      val badPipeline = Pipeline(badJson)
      badPipeline.validate() should be (false)
      badPipeline.dispose()
      badPipeline.ptr should be (0)
    }

    it("should validate json") {
      pipeline.validate() should be (true)
    }

    it("should execute pipeline") {
      pipeline.execute()
    }

    it("should create pointViews iterator") {
      val pvi = pipeline.pointViews()
      pvi.length should be (1)
      pvi.dispose()
    }

    it("should have a valid point view size") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.length should be (1065)
      pvi.hasNext should be (false)
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid (X, Y, Z) data") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.getX(0) should be (637012.24)
      pv.getY(0) should be (849028.31)
      pv.getZ(0) should be (431.66)
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid packed data") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      val layout = pv.layout
      val arr = pv.getPackedPoint(Array(DimType.X, DimType.Y), 0)
      val (xarr, yarr) = arr.take(layout.dimSize(DimType.X).toInt) -> arr.drop(layout.dimSize(DimType.Y).toInt)

      ByteBuffer.wrap(xarr).order(ByteOrder.LITTLE_ENDIAN).getDouble should be (pv.getX(0))
      ByteBuffer.wrap(yarr).order(ByteOrder.LITTLE_ENDIAN).getDouble should be (pv.getY(0))

      layout.dispose()
      pv.dispose()
      pvi.dispose()
    }

    it("should read the whole packedpoint and grab only one dim") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      val arr = pv.getPackedPoint(0)
      pv.get(arr)(DimType.Y).getDouble should be (pv.getY(0))
      pv.dispose()
      pvi.dispose()
    }

    it("should read a valid value by name") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.get("ReturnNumber", 0).get() & 0xff should be (1)
      pv.dispose()
      pvi.dispose()
    }

    it("layout should have a valid number of dims") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.layout.dimTypes().length should be (16)
      pv.dispose()
      pvi.dispose()
    }

    it("should find a dim by name") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.findDimType("Red") should be (DimType("Red", "uint16_t"))
      pv.dispose()
      pvi.dispose()
    }

    it("dim sizes should be of a valid size") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      val layout = pv.layout
      layout.dimTypes().map(pv.layout.dimSize(_)).sum should be (layout.pointSize())
      layout.dispose()
      pv.dispose()
      pvi.dispose()
    }

    it("should read all packed points valid") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      val layout = pv.layout
      pv.getPackedPoints.length should be (pv.length * layout.pointSize())
      layout.dispose()
      pv.dispose()
      pvi.dispose()
    }

    it("should read crs correct") {
      val pvi = pipeline.pointViews()
      val pv = pvi.next()
      pv.getCrsProj4 should be (proj4String)
      pv.dispose()
      pvi.dispose()
    }
  }

  override def afterAll() = {
    pipeline.dispose()
  }
}
