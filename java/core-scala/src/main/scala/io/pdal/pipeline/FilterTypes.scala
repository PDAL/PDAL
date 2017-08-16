/******************************************************************************
  * Copyright (c) 2017, hobu Inc.  (info@hobu.co)
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

package io.pdal.pipeline

sealed trait FilterType extends ExprType { val `type` = "filters" }

object FilterTypes {
  case object approximatecoplanar extends FilterType
  case object chipper extends FilterType
  case object cluster extends FilterType
  case object colorinterp extends FilterType
  case object colorization extends FilterType
  case object computerange extends FilterType
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
  case object hexbin extends FilterType
  case object iqr extends FilterType
  case object kdistance extends FilterType
  case object locate extends FilterType
  case object lof extends FilterType
  case object mad extends FilterType
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
  case object predicate extends FilterType
  case object programmable extends FilterType
  case object radialdensity extends FilterType
  case object range extends FilterType
  case object randomize extends FilterType
  case object reprojection extends FilterType
  case object sample extends FilterType
  case object smrf extends FilterType
  case object sort extends FilterType
  case object splitter extends FilterType
  case object stats extends FilterType
  case object transformation extends FilterType
  case object voxelgrid extends FilterType

  lazy val all = List(
    approximatecoplanar, chipper, cluster, colorinterp, colorization, computerange,
    crop, decimation, divider, eigenvalues, estimaterank, ferry, greedyprojection, gridprojection, groupby,
    hag, hexbin, iqr, kdistance, locate, lof, mad, merge, mongus, mortonorder, movingleastsquares, normal, outlier,
    overlay, pclblock, pmf, poisson, predicate, programmable, radialdensity, randomize, range, reprojection,
    sample, smrf, sort, splitter, stats, transformation, voxelgrid
  )

  def fromName(name: String): FilterType =
    all.find(_.name == name).getOrElse(throw new Exception(s"FilterType $name is not supported."))
}