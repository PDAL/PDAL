#include <jni.h>
#include <string>

#ifndef _ARRAYLIST_H_INCLUDED_
#define _ARRAYLIST_H_INCLUDED_

namespace libpdaljava
{
class ArrayList
{
public:
    jclass arrayClass;
    jmethodID arrayCtor;
    jmethodID arraySize;
    jmethodID arrayGet;
    jmethodID arrayAdd;

    ArrayList(JNIEnv *);
    ~ArrayList();
};
}
#endif