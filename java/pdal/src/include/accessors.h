#ifndef _HANDLE_H_INCLUDED_
#define _HANDLE_H_INCLUDED_

jfieldID getHandleField(JNIEnv *env, jobject obj)
{
    jclass c = env->GetObjectClass(obj);
    // J is the type signature for long:
    return env->GetFieldID(c, "nativeHandle", "J");
}

template <typename T>
T *getHandle(JNIEnv *env, jobject obj)
{
    jlong handle = env->GetLongField(obj, getHandleField(env, obj));
    return reinterpret_cast<T *>(handle);
}

template <typename T>
void setHandle(JNIEnv *env, jobject obj, T *t)
{
    jlong handle = reinterpret_cast<jlong>(t);
    env->SetLongField(obj, getHandleField(env, obj), handle);
}

std::string getJson(JNIEnv *env, jobject obj)
{
    jclass c = env->GetObjectClass(obj);
    jfieldID fid = env->GetFieldID(c, "json", "Ljava/lang/String;");
    jstring jstr = (jstring) env->GetObjectField(obj, fid);
    const char *str = env->GetStringUTFChars(jstr, 0);
    return std::string(str);
}

#endif