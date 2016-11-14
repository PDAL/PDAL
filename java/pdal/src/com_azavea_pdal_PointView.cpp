#include <stdio.h>
#include "com_azavea_pdal_PointView.h"
#include "JavaPipeline.hpp"
#include "Accessors.hpp"

using libpdaljava::Pipeline;
using pdal::PointView;
using pdal::PointLayoutPtr;

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_PointView_layout
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    PointLayoutPtr pl = pv->layout();

    jclass pvlClass = env->FindClass("com/azavea/pdal/PointLayout");
    jmethodID pvlCtor = env->GetMethodID(pvlClass, "<init>", "()V");
    jobject pvl = env->NewObject(pvlClass, pvlCtor);

    setHandle(env, pvl, pl);

    return pvl;
}

JNIEXPORT jint JNICALL Java_com_azavea_pdal_PointView_size
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    return pv->size();
}

JNIEXPORT jboolean JNICALL Java_com_azavea_pdal_PointView_empty
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    return pv->empty();
}

JNIEXPORT jobject JNICALL Java_com_azavea_pdal_PointView_points
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    //return;
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_PointView_dispose
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    setHandle<int>(env, obj, 0);
    delete pv;
}

JNIEXPORT void JNICALL Java_com_azavea_pdal_PointView_test
  (JNIEnv *env, jobject obj)
{
    PointView *pv = getHandle<PointView>(env, obj);
    std::cout << pv->size() << std::endl;
}
