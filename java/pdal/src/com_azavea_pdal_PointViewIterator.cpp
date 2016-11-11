#include <stdio.h>
#include "com_azavea_pdal_PointViewIterator.h"
#include "JavaPipeline.hpp"
#include "Accessors.h"

using pdal::PointViewSet;
using pdal::PointView;
using pdal::PointViewPtr;

JNIEXPORT jboolean JNICALL Java_com_azavea_pdal_PointViewIterator_hasNext
  (JNIEnv *env, jobject obj)
{
  PointViewSet *pvset = getHandle<PointViewSet>(env, obj);

  std::set<PointViewPtr>::iterator itb = pvset->begin();
  std::set<PointViewPtr>::iterator ite = pvset->end();

  return itb == ite;
}

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_PointViewIterator_next
  (JNIEnv *env, jobject obj)
{
  //PointViewSet *pvset = getHandle<PointViewSet>(env, obj);
  //PointViewPtr pvsetptr = pvset->begin();
  //setHandle(env, pvset, ++&*pvsetptr); // move own iterator forward
}
