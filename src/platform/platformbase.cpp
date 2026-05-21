#include <cstdarg>
#include <cstdio>

#include <mimalloc.h>

#if defined(WIN32)
// Lowercase form so case-sensitive cross-builds (MinGW-w64 on Linux)
// find it; MSVC/Windows is case-insensitive and accepts both.
#   include <windows.h>
#endif // defined(WIN32)

#include "util.h"
#include "platform.h"

namespace SolveSpace {
namespace Platform {

//-----------------------------------------------------------------------------
// Debug output, on Windows.
//-----------------------------------------------------------------------------

#if defined(WIN32)

#if !defined(_alloca)
// Fix for compiling with MinGW.org GCC-6.3.0-1
#define _alloca alloca
#include <malloc.h>
#endif

void DebugPrint(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int len = _vscprintf(fmt, va) + 1;
    va_end(va);

    va_start(va, fmt);
    char *buf = (char *)_alloca(len);
    _vsnprintf(buf, len, fmt, va);
    va_end(va);

    // The native version of OutputDebugString, unlike most others,
    // is OutputDebugStringA.
    OutputDebugStringA(buf);
    OutputDebugStringA("\n");

#ifndef NDEBUG
    // Duplicate to stderr in debug builds, but not in release; this is slow.
    fputs(buf, stderr);
    fputc('\n', stderr);
#endif
}

#endif

//-----------------------------------------------------------------------------
// Debug output, on *nix.
//-----------------------------------------------------------------------------

#if !defined(WIN32)

void DebugPrint(const char *fmt, ...) {
    va_list va;
    va_start(va, fmt);
    vfprintf(stderr, fmt, va);
    fputc('\n', stderr);
    va_end(va);
}

#endif

//-----------------------------------------------------------------------------
// Temporary arena.
//
// Per-thread scratch heap used by `AllocExpr()` (see expr.cpp). It's a
// workspace, not state — every solve allocates a fan of Expr nodes,
// runs Newton's method, and the caller (the `Slvs_*` C API) issues
// `FreeAllTemporary` at the end of the solve. Threads never share an
// arena because the heap pointer is `thread_local`. There is no
// dependency on which Solver is being driven: two Solvers running
// on the same thread (in sequence) reuse the same arena, freed
// between solves; two Solvers running on different threads each
// get their own.
//
// Lifetime: lazy-allocated on first `AllocTemporary`, destroyed at
// thread exit via the `Arena` RAII helper below. `FreeAllTemporary`
// also tears it down explicitly mid-solve when the caller chooses.
//-----------------------------------------------------------------------------

namespace {
struct Arena {
    mi_heap_t *heap = nullptr;
    ~Arena() {
        if(heap != nullptr) {
            mi_heap_destroy(heap);
        }
    }
};
thread_local Arena arena;
}  // namespace

void *AllocTemporary(size_t size) {
    if(arena.heap == nullptr) {
        arena.heap = mi_heap_new();
        ssassert(arena.heap != nullptr, "out of memory");
    }
    void *ptr = mi_heap_zalloc(arena.heap, size);
    ssassert(ptr != nullptr, "out of memory");
    return ptr;
}

void FreeAllTemporary() {
    if(arena.heap != nullptr) {
        mi_heap_destroy(arena.heap);
        arena.heap = nullptr;
    }
}

}
}
