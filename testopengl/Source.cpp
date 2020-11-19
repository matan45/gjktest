#include <jni.h>

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM * vm, void* reserved) {
	return JNI_VERSION_10;
}