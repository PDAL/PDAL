/*
 * Copyright 2016 Azavea
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package geotrellis.pointcloud.pipeline

import io.circe.Json

sealed trait PipelineExpr {
  def ~(other: PipelineExpr): PipelineConstructor =
    PipelineConstructor(this :: other :: Nil)

  def ~(other: Option[PipelineExpr]): PipelineConstructor =
    other.fold(PipelineConstructor(this :: Nil))(o => PipelineConstructor(this :: o :: Nil))
}

case class RawExpr(json: Json) extends PipelineExpr

case class Read(
  filename: String,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: Option[ReaderType] = None // usually auto derived by pdal
) extends PipelineExpr

case class FauxRead(
  numPoints: Int,
  mode: String, // constant | random | ramp | uniform | normal
  stdevX: Option[Int] = None, // [default: 1]
  stdevY: Option[Int] = None, // [default: 1]
  stdevZ: Option[Int] = None, // [default: 1]
  meanX: Option[Int] = None, // [default: 0]
  meanY: Option[Int] = None, // [default: 0]
  meanZ: Option[Int] = None, // [default: 0]
  bounds: Option[String] = None, // [default: unit cube]
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.faux
) extends PipelineExpr

object GdalRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.gdal))
}

case class GeoWaveRead(
  zookeeperUrl: String,
  instanceName: String,
  username: String,
  password: String,
  tableNamespace: String,
  featureTypeName: Option[String] = None, // [default: PDAL_Point]
  dataAdapter: Option[String] = None, // [default: FeatureCollectionDataAdapter]
  pointsPerEntry: Option[String] = None, // [default: 5000u]
  bounds: Option[String] = None, // [default: unit cube]
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.geowave
) extends PipelineExpr

case class GreyhoundRead(
  url: String,
  bounds: Option[String] = None, // [default: the entire resource]
  depthBegin: Option[Int] = None, // [default: 0]
  depthEnd: Option[Int] = None, // [default: 0]
  tilePath: Option[String] = None,
  filter: Option[Json] = None,
  threads: Option[Int] = None, // [default: 4]
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.greyhound
) extends PipelineExpr

case class Ilvis2Read(
  filename: String,
  mapping: Option[String] = None,
  metadata: Option[String] = None,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.ilvis2
) extends PipelineExpr

case class LasRead(
  filename: String,
  extraDims: Option[String] = None,
  compression: Option[String] = None,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.las
) extends PipelineExpr

object MrsidRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.mrsid))
}

object NitfRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.nitf))
}

case class OciRead(
  connection: String,
  query: String,
  xmlSchemaDump: Option[String] = None,
  populatePointsourceid: Option[String] = None,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.oci
) extends PipelineExpr

object OptechRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.optech))
}

object PcdRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.pcd))
}

case class PgpointcloudRead(
  connection: String,
  table: String,
  schema: Option[String] = None, // [default: public]
  column: Option[String] = None, // [default: pa]
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.pgpointcloud
) extends PipelineExpr

object PlyRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.ply))
}

object PtsRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.pts))
}

case class QfitRead(
  filename: String,
  flipCoordinates: Option[Boolean] = None,
  scaleZ: Option[Double] = None,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.qfit
) extends PipelineExpr

case class RxpRead(
  filename: String,
  rdtp: Option[Boolean] = None,
  syncToPps: Option[Boolean] = None,
  minimal: Option[Boolean] = None,
  reflectanceAsIntensity: Option[Boolean] = None,
  minReflectance: Option[Double] = None,
  maxReflectance: Option[Double] = None,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.rxp
) extends PipelineExpr

object SbetRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.sbet))
}

case class SqliteRead(
  connection: String,
  query: String,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.sqlite
) extends PipelineExpr

object TextRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.text))
}

case class TindexRead(
  filename: String,
  layerName: Option[String] = None,
  srsColumn: Option[String] = None,
  tindexName: Option[String] = None,
  sql: Option[String] = None,
  wkt: Option[String] = None,
  boundary: Option[String] = None,
  tSrs: Option[String] = None,
  filterSrs: Option[String] = None,
  where: Option[String] = None,
  dialect: Option[String] = None,
  spatialreference: Option[String] = None,
  tag: Option[String] = None,
  `type`: ReaderType = ReaderTypes.tindex
) extends PipelineExpr

object TerrasolidRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.terrasolid))
}

object IceBridgeRead {
  def apply(filename: String, spatialreference: Option[String] = None, tag: Option[String] = None): Read =
    Read(filename, spatialreference, tag, Some(ReaderTypes.icebridge))
}

case class ApproximateCoplanarFilter(
  knn: Int, // [default: 8]
  thresh1: Int, // [default: 25]
  thresh2: Int, // [default: 6]
  `type`: FilterType = FilterTypes.approximatecoplanar
) extends PipelineExpr

case class AttributeFilter(
  dimension: Option[String] = None, // [default: none]
  value: Option[Double] = None, // [default: none]
  datasource: Option[String] = None, // [default: none]
  column: Option[String] = None, // [default: none]
  query: Option[String] = None, // [default: first column]
  layer: Option[String] = None, // [default: first layer]
  `type`: FilterType = FilterTypes.attribute
) extends PipelineExpr

case class ChipperFilter(
  capacity: Option[Int] = None, // [default: 5000]
  `type`: FilterType = FilterTypes.chipper
) extends PipelineExpr

case class ColorinterpFilter(
  ramp: Option[String] = None, // [default: pestel_shades]
  dimension: Option[String] = None, // [default: Z]
  minimum: Option[String] = None,
  maximum: Option[String] = None,
  invert: Option[Boolean] = None, // [default: false]
  k: Option[Double] = None,
  mad: Option[Boolean] = None,
  madMultiplier: Option[Double] = None,
  `type`: FilterType = FilterTypes.colorinterp
) extends PipelineExpr

case class ColorizationFilter(
  raster: String,
  dimensions: Option[String] = None,
  `type`: FilterType = FilterTypes.colorization
) extends PipelineExpr

case class ComputerangeFilter(
  `type`: FilterType = FilterTypes.computerange
) extends PipelineExpr

case class CropFilter(
  bounds: Option[String] = None,
  polygon: Option[String] = None,
  outside: Option[String] = None,
  point: Option[String] = None,
  radius: Option[String] = None,
  `type`: FilterType = FilterTypes.crop
) extends PipelineExpr

case class DecimationFilter(
  step: Option[Int] = None,
  offset: Option[Int] = None,
  limit: Option[Int] = None,
  `type`: FilterType = FilterTypes.decimation
) extends PipelineExpr

case class DividerFilter(
   mode: Option[String] = None,
   count: Option[Int] = None,
   capacity: Option[Int] = None,
  `type`: FilterType = FilterTypes.divider
) extends PipelineExpr

case class EigenValuesFilter(
  knn: Option[Int] = None,
  `type`: FilterType = FilterTypes.eigenvalues
) extends PipelineExpr

case class EstimateRankFilter(
  knn: Option[Int] = None,
  thresh: Option[Double] = None,
  `type`: FilterType = FilterTypes.estimaterank
) extends PipelineExpr

case class FerryFilter(
  dimensions: String,
  `type`: FilterType = FilterTypes.ferry
) extends PipelineExpr

case class GreedyProjectionFilter(
  `type`: FilterType = FilterTypes.greedyprojection
) extends PipelineExpr

case class GridProjectionFilter(
  `type`: FilterType = FilterTypes.gridprojection
) extends PipelineExpr

case class HagFilter(
  `type`: FilterType = FilterTypes.hag
) extends PipelineExpr

case class HexbinFilter(
  edgeSize: Option[Int] = None,
  sampleSize: Option[Int] = None,
  threshold: Option[Int] = None,
  precision: Option[Int] = None,
  `type`: FilterType = FilterTypes.hexbin
) extends PipelineExpr

case class IqrFilter(
  dimension: String,
  k: Option[Double] = None,
  `type`: FilterType = FilterTypes.iqr
) extends PipelineExpr

case class KDistanceFilter(
  k: Option[Int] = None,
  `type`: FilterType = FilterTypes.kdistance
) extends PipelineExpr

case class LofFilter(
  minpts: Option[Int] = None,
  `type`: FilterType = FilterTypes.lof
) extends PipelineExpr

case class MadFilter(
  dimension: String,
  k: Option[Double] = None,
  `type`: FilterType = FilterTypes.mad
) extends PipelineExpr

case class MergeFilter(
  inputs: List[String],
  tag: Option[String] = None,
  `type`: FilterType = FilterTypes.merge
) extends PipelineExpr

case class MongusFilter(
  cell: Option[Double] = None,
  classify: Option[Boolean] = None,
  extract: Option[Boolean] = None,
  k: Option[Double] = None,
  l: Option[Int] = None,
  `type`: FilterType = FilterTypes.mongus
) extends PipelineExpr

case class MortnOrderFilter(
  `type`: FilterType = FilterTypes.mortonorder
) extends PipelineExpr

case class MovingLeastSquaresFilter(
  `type`: FilterType = FilterTypes.movingleastsquares
) extends PipelineExpr

case class NormalFilter(
  knn: Option[Int] = None,
  `type`: FilterType = FilterTypes.normal
) extends PipelineExpr

case class OutlierFilter(
  method: Option[String] = None,
  minK: Option[Int] = None,
  radius: Option[Double] = None,
  meanK: Option[Int] = None,
  multiplier: Option[Double] = None,
  classify: Option[Boolean] = None,
  extract: Option[Boolean] = None,
  `type`: FilterType = FilterTypes.outlier
) extends PipelineExpr

case class PclBlockFilter(
  filename: String,
  methods: Option[List[String]] = None,
  `type`: FilterType = FilterTypes.pclblock
) extends PipelineExpr

case class PmfFilter(
  maxWindowSize: Option[Int] = None,
  slope: Option[Double] = None,
  maxDistance: Option[Double] = None,
  initialDistance: Option[Double] = None,
  cellSize: Option[Int] = None,
  classify: Option[Boolean] = None,
  extract: Option[Boolean] = None,
  approximate: Option[Boolean] = None,
  `type`: FilterType = FilterTypes.pmf
) extends PipelineExpr

case class PoissonFilter(
  depth: Option[Int] = None,
  pointWeight: Option[Double] = None,
  `type`: FilterType = FilterTypes.poisson
) extends PipelineExpr

case class PredicateFilter(
  script: String,
  module: String,
  function: String,
  `type`: FilterType = FilterTypes.predicate
) extends PipelineExpr

case class ProgrammableFilter(
  script: String,
  module: String,
  function: String,
  source: String,
  addDimenstion: Option[String] = None,
  `type`: FilterType = FilterTypes.programmable
) extends PipelineExpr

case class RadialDensityFilter(
  radius: Option[Double] = None,
  `type`: FilterType = FilterTypes.radialdensity
) extends PipelineExpr

case class RandomizeFilter(
  `type`: FilterType = FilterTypes.randomize
) extends PipelineExpr

case class RangeFilter(
  limits: Option[String] = None,
  `type`: FilterType = FilterTypes.range
) extends PipelineExpr

case class ReprojectionFilter(
  outSrs: String,
  inSrs: Option[String] = None,
  tag: Option[String] = None,
  `type`: FilterType = FilterTypes.reprojection
) extends PipelineExpr

case class SampleFilter(
  radius: Option[Double] = None,
  `type`: FilterType = FilterTypes.sample
) extends PipelineExpr

case class SmrfFilter(
  cell: Option[Double] = None,
  classify: Option[Boolean] = None,
  cut: Option[Double] = None,
  extract: Option[Boolean] = None,
  slope: Option[Double] = None,
  threshold: Option[Double] = None,
  window: Option[Double] = None,
  `type`: FilterType = FilterTypes.smrf
) extends PipelineExpr

case class SortFilter(
  dimension: String,
  `type`: FilterType = FilterTypes.sort
) extends PipelineExpr

case class SplitterFilter(
  length: Option[Int] = None,
  originX: Option[Double] = None,
  originY: Option[Double] = None,
  `type`: FilterType = FilterTypes.splitter
) extends PipelineExpr

case class StatsFilter(
  dimenstions: Option[String] = None,
  enumerate: Option[String] = None,
  count: Option[Int] = None,
  `type`: FilterType = FilterTypes.stats
) extends PipelineExpr

case class TransformationFilter(
  matrix: String,
  `type`: FilterType = FilterTypes.transformation
) extends PipelineExpr

case class VoxelGridFilter(
  leafX: Option[Double] = None,
  leafY: Option[Double] = None,
  leafZ: Option[Double] = None,
  `type`: FilterType = FilterTypes.voxelgrid
) extends PipelineExpr

case class Write(
  filename: String,
  `type`: Option[WriterType] = None // usually auto derived by pdal
) extends PipelineExpr

case class BpfWrite(
  filename: String,
  compression: Option[Boolean] = None,
  format: Option[String] = None,
  bundledfile: Option[String] = None,
  headerData: Option[String] = None,
  coordId: Option[Int] = None,
  scaleX: Option[Double] = None,
  scaleY: Option[Double] = None,
  scaleZ: Option[Double] = None,
  offsetX: Option[String] = None,
  offsetY: Option[String] = None,
  offsetZ: Option[String] = None,
  outputDims: Option[String] = None,
  `type`: WriterType = WriterTypes.bpf
) extends PipelineExpr

case class DerivativeWrite(
  filename: String,
  primitiveType: Option[String] = None,
  edgeLength: Option[Double] = None,
  altitude: Option[Double] = None,
  azimuth: Option[Double] = None,
  `type`: WriterType = WriterTypes.derivative
) extends PipelineExpr

case class GdalWrite(
  filename: String,
  resoultion: Int,
  radius: Double,
  gdaldriver: Option[String] = None,
  gdalopts: Option[String] = None,
  outputType: Option[String] = None,
  windowSize: Option[Int] = None,
  dimension: Option[String] = None,
  `type`: WriterType = WriterTypes.gdal
) extends PipelineExpr

case class GeoWaveWrite(
  zookeeperUrl: String,
  instanceName: String,
  username: String,
  password: String,
  tableNamespace: String,
  featureTypeName: Option[String] = None,
  dataAdapter: Option[String] = None,
  pointsPerEntry: Option[String] = None, // [default: 5000u]
  `type`: WriterType = WriterTypes.geowave
) extends PipelineExpr

case class LasWrite(
  filename: String,
  forward: Option[String] = None,
  minorVersion: Option[Int] = None,
  softwareId: Option[String] = None,
  creationDoy: Option[Int] = None,
  creationYear: Option[Int] = None,
  dataformatId: Option[Int] = None,
  systemId: Option[String] = None,
  aSrs: Option[String] = None,
  globalEncoding: Option[String] = None,
  projectId: Option[String] = None,
  compression: Option[String] = None,
  scaleX: Option[Double] = None,
  scaleY: Option[Double] = None,
  scaleZ: Option[Double] = None,
  offsetX: Option[String] = None,
  offsetY: Option[String] = None,
  offsetZ: Option[String] = None,
  filesourceId: Option[Int] = None,
  discardHighReturnNumbers: Option[Boolean] = None,
  `type`: WriterType = WriterTypes.las
) extends PipelineExpr

case class MatlabWrite(
  filename: String,
  outputDims: Option[String] = None,
  `type`: WriterType = WriterTypes.matlab
) extends PipelineExpr

case class NitfWrite(
  filename: String,
  clevel: Option[String] = None,
  stype: Option[String] = None,
  ostaid: Option[String] = None,
  ftitle: Option[String] = None,
  fscalas: Option[String] = None,
  oname: Option[String] = None,
  ophone: Option[String] = None,
  fsctlh: Option[String] = None,
  fsclsy: Option[String] = None,
  idatim: Option[String] = None,
  iid2: Option[String] = None,
  fscltx: Option[String] = None,
  aimidb: Option[String] = None,
  acftb: Option[String] = None,
  `type`: WriterType = WriterTypes.nitf
) extends PipelineExpr

case class NullWrite(
  `type`: WriterType = WriterTypes.`null`
) extends PipelineExpr

case class OciWrite(
  connection: String,
  is3d: Option[Boolean] = None,
  solid: Option[Boolean] = None,
  overwrite: Option[Boolean] = None,
  verbose: Option[Boolean] = None,
  srid: Option[Int] = None,
  capacity: Option[Int] = None,
  streamOutputPrecision: Option[Int] = None,
  cloudId: Option[Int] = None,
  blockTableName: Option[String] = None,
  blockTablePartitionValue: Option[Int] = None,
  baseTableName: Option[String] = None,
  cloudColumnName: Option[String] = None,
  baseTableAuxColumns: Option[String] = None,
  baseTableAuxValues: Option[String] = None,
  baseTableBoundaryColumn: Option[String] = None,
  baseTableBoundaryWkt: Option[String] = None,
  preBlockSql: Option[String] = None,
  preSql: Option[String] = None,
  postBlockSql: Option[String] = None,
  baseTableBounds: Option[String] = None,
  pcId: Option[Int] = None,
  packIgnoredFields: Option[Boolean] = None,
  streamChunks: Option[Boolean] = None,
  blobChunkCount: Option[Int] = None,
  scaleX: Option[Double] = None,
  scaleY: Option[Double] = None,
  scaleZ: Option[Double] = None,
  offsetX: Option[Double] = None,
  offsetY: Option[Double] = None,
  offsetZ: Option[Double] = None,
  outputDims: Option[String] = None,
  `type`: WriterType = WriterTypes.oci
) extends PipelineExpr

case class P2gWrite(
  filename: String,
  gridDistX: Option[Int] = None,
  gridDistY: Option[Int] = None,
  radiuse: Option[Double] = None,
  outputType: Option[String] = None,
  outputFormat: Option[String] = None,
  z: Option[String] = None,
  bounds: Option[String] = None,
  `type`: WriterType = WriterTypes.p2g
) extends PipelineExpr

case class PcdWrite(
  filename: String,
  compression: Option[Boolean] = None,
  `type`: WriterType = WriterTypes.pcd
) extends PipelineExpr

case class PgpointcloudWrite(
  connection: String,
  table: String,
  schema: Option[String] = None,
  column: Option[String] = None,
  compression: Option[String] = None,
  overwrite: Option[Boolean] = None,
  srid: Option[Int] = None,
  pcid: Option[Int] = None,
  preSql: Option[String] = None,
  postSql: Option[String] = None,
  scaleX: Option[Double] = None,
  scaleY: Option[Double] = None,
  scaleZ: Option[Double] = None,
  offsetX: Option[Double] = None,
  offsetY: Option[Double] = None,
  offsetZ: Option[Double] = None,
  outputDims: Option[String] = None,
  `type`: WriterType = WriterTypes.pgpointcloud
) extends PipelineExpr

case class PlyWrite(
  filename: String,
  storageMode: Option[String] = None,
  `type`: WriterType = WriterTypes.ply
) extends PipelineExpr

case class RialtoWrite(
  filename: String,
  maxLevels: Option[Int] = None,
  overwrite: Option[Boolean] = None,
  `type`: WriterType = WriterTypes.rialto
) extends PipelineExpr

case class SqliteWrite(
  filename: String,
  cloudTableName: String,
  blockTableName: String,
  cloudColumnName: Option[String] = None,
  compression: Option[String] = None,
  overwrite: Option[Boolean] = None,
  preSql: Option[String] = None,
  postSql: Option[String] = None,
  scaleX: Option[Double] = None,
  scaleY: Option[Double] = None,
  scaleZ: Option[Double] = None,
  offsetX: Option[Double] = None,
  offsetY: Option[Double] = None,
  offsetZ: Option[Double] = None,
  outputDims: Option[String] = None,
  `type`: WriterType = WriterTypes.sqlite
) extends PipelineExpr

case class TextWrite(
  filename: String,
  format: Option[String] = None,
  order: Option[String] = None,
  keepUnspecified: Option[Boolean] = None,
  jscallback: Option[String] = None,
  quoteHeader: Option[String] = None,
  newline: Option[String] = None,
  delimiter: Option[String] = None,
  `type`: WriterType = WriterTypes.text
) extends PipelineExpr

case class PipelineConstructor(list: List[PipelineExpr] = Nil) extends PipelineExpr {
  override def ~(e: PipelineExpr): PipelineConstructor = PipelineConstructor(list :+ e)
  override def ~(e: Option[PipelineExpr]): PipelineConstructor = e.fold(this)(el => PipelineConstructor(list :+ el))
  def map[B](f: PipelineExpr => B): List[B] = list.map(f)
  def mapExpr(f: PipelineExpr => PipelineExpr): PipelineConstructor = PipelineConstructor(list.map(f))
}
