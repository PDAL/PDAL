/*
 * Copyright 2017 Azavea
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

package io.pdal.pipeline

sealed trait FilterType extends ExprType { val `type` = "filters" }

object FilterTypes {
  case object approximatecoplanar extends FilterType
  case object chipper extends FilterType
  case object cluster extends FilterType
  case object colorinterp extends FilterType
  case object colorization extends FilterType
  case object computerange extends FilterType
  case object cpd extends FilterType
  case object crop extends FilterType
  case object decimation extends FilterType
  case object divider extends FilterType
  case object eigenvalues extends FilterType
  case object estimaterank extends FilterType
  case object ferry extends FilterType
  case object greedyprojection extends FilterType
  case object gridprojection extends FilterType
  case object groupby extends FilterType
  case object hag extends FilterType
  case object head extends FilterType
  case object hexbin extends FilterType
  case object icp extends FilterType
  case object iqr extends FilterType
  case object kdistance extends FilterType
  case object locate extends FilterType
  case object lof extends FilterType
  case object mad extends FilterType
  case object matlab extends FilterType
  case object merge extends FilterType
  case object mongus extends FilterType
  case object mortonorder extends FilterType
  case object movingleastsquares extends FilterType
  case object normal extends FilterType
  case object overlay extends FilterType
  case object outlier extends FilterType
  case object pclblock extends FilterType
  case object pmf extends FilterType
  case object poisson extends FilterType
  case object python extends FilterType
  case object radialdensity extends FilterType
  case object range extends FilterType
  case object randomize extends FilterType
  case object reprojection extends FilterType
  case object sample extends FilterType
  case object smrf extends FilterType
  case object sort extends FilterType
  case object splitter extends FilterType
  case object stats extends FilterType
  case object tail extends FilterType
  case object transformation extends FilterType
  case object voxelcenternearestneighbor extends FilterType
  case object voxelcentroidnearestneighbor extends FilterType
  case object voxelgrid extends FilterType

  lazy val all = List(
    approximatecoplanar, chipper, cluster, colorinterp, colorization, computerange, crop,
    cpd, decimation, divider, eigenvalues, estimaterank, ferry, greedyprojection, gridprojection, groupby,
    hag, head, hexbin, icp, iqr, kdistance, locate, lof, mad, matlab, merge, mongus, mortonorder, movingleastsquares,
    normal, outlier, overlay, pclblock, pmf, poisson, python, radialdensity, randomize, range, reprojection,
    sample, smrf, sort, splitter, stats, transformation, voxelcenternearestneighbor, voxelcentroidnearestneighbor,
    voxelgrid
  )

  def fromName(name: String): FilterType =
    all.find(_.name == name).getOrElse(throw new Exception(s"FilterType $name is not supported."))
}