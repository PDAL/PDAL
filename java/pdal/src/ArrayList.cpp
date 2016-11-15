#include "ArrayList.hpp"

namespace libpdaljava
{
ArrayList::ArrayList(JNIEnv *env)
{
    arrayClass = env->FindClass("java/util/ArrayList");
    arrayCtor = env->GetMethodID(arrayClass, "<init>", "(I)V");
    arraySize = env->GetMethodID(arrayClass, "size", "()I");
    arrayGet = env->GetMethodID(arrayClass, "get", "(I)Ljava/lang/Object;");
    arrayAdd = env->GetMethodID(arrayClass, "add", "(Ljava/lang/Object;)Z");
}

ArrayList::~ArrayList()
{ }
}