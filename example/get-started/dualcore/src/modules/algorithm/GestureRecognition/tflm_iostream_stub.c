/* Keeps libc++'s iostream/locale machinery (~110 KB flash) out of the image.
 *
 * TFLite-Micro TUs (kernel_util / lstm_eval / mul_common) transitively
 * include <iostream>; libc++ plants a per-TU static std::ios_base::Init
 * object whose constructor lives in iostream.cpp.o, so armlink pulls
 * iostream.cpp.o -> ios.cpp.o -> locale.cpp.o even though nothing in this
 * firmware uses std streams (verified via main.map: zero cout/cerr/cin
 * references). Defining the mangled ctor/dtor symbols here resolves those
 * references locally as no-ops and the whole chain is dropped.
 *
 * If real std::cout/cerr usage is ever introduced, DELETE THIS FILE —
 * otherwise the streams would stay uninitialized.
 */

/* std::__1::ios_base::Init::Init() */
void _ZNSt3__18ios_base4InitC2Ev(void *self) { (void)self; }
void _ZNSt3__18ios_base4InitC1Ev(void *self) { (void)self; }
/* std::__1::ios_base::Init::~Init() */
void _ZNSt3__18ios_base4InitD2Ev(void *self) { (void)self; }
void _ZNSt3__18ios_base4InitD1Ev(void *self) { (void)self; }
