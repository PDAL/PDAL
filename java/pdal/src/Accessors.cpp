#include <jni.h>
#include <string>

jfieldID getHandleField(JNIEnv *env, jobject obj)
{
    jclass c = env->GetObjectClass(obj);
    // J is the type signature for long:
    return env->GetFieldID(c, "nativeHandle", "J");
}

std::string getJson(JNIEnv *env, jobject obj)
{
    jclass c = env->GetObjectClass(obj);
    jfieldID fid = env->GetFieldID(c, "json", "Ljava/lang/String;");
    jstring jstr = (jstring) env->GetObjectField(obj, fid);
    const char *str = env->GetStringUTFChars(jstr, 0);
    return std::string(str);
}
