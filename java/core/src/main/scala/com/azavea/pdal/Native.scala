package com.azavea.pdal

trait Native {
  protected var nativeHandle = 0l // C++ pointer
  def ptr: Long = nativeHandle
}
