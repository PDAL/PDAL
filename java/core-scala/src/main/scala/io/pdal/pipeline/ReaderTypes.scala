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

sealed trait ReaderType extends ExprType { val `type` = "readers" }

object ReaderTypes {
  case object bpf extends ReaderType
  case object buffer extends ReaderType
  case object faux extends ReaderType
  case object gdal extends ReaderType
  case object geowave extends ReaderType
  case object greyhound extends ReaderType
  case object ilvis2 extends ReaderType
  case object mbio extends ReaderType
  case object las extends ReaderType
  case object mrsid extends ReaderType
  case object nitf extends ReaderType
  case object oci extends ReaderType
  case object optech extends ReaderType
  case object pcd extends ReaderType
  case object pgpointcloud extends ReaderType
  case object ply extends ReaderType
  case object pts extends ReaderType
  case object qfit extends ReaderType
  case object rxp extends ReaderType
  case object sbet extends ReaderType
  case object sqlite extends ReaderType
  case object text extends ReaderType
  case object tindex extends ReaderType
  case object terrasolid extends ReaderType
  case object icebridge extends ReaderType

  lazy val all = List(
    bpf, buffer, faux, gdal, geowave, greyhound, ilvis2, mbio, las, mrsid, nitf,
    oci, optech, pcd, pgpointcloud, ply, pts, qfit, rxp, sbet, sqlite, text,
    tindex, terrasolid, icebridge
  )

  def fromName(name: String): ReaderType =
    all.find(_.name == name).getOrElse(throw new Exception(s"ReaderType $name is not supported."))
}