#include "debug.h"

#ifdef WIN32
#include <intrin.h>
#endif

void MaybeDebugBreak() {
#ifndef NDEBUG
  #ifdef WIN32
    __debugbreak();
  #endif
#endif
}

