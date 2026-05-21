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
// Pulls in the Solver type + EnsureCurrentSolver() used below by
// AllocTemporary / FreeAllTemporary.
#include "solvespace.h"

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
// The per-solve scratch heap used by AllocExpr() (see expr.cpp) lives
// inside the current `Solver` (see solver.h). One heap per Solver
// instance — single-threaded callers see the same lazy thread-local
// behaviour as before; the handle-based API (Phase 0.5+) lets a single
// thread juggle several Solvers, each with its own heap.
//-----------------------------------------------------------------------------

void *AllocTemporary(size_t size) {
    Solver &s = EnsureCurrentSolver();
    if(s.temp_heap == nullptr) {
        s.temp_heap = mi_heap_new();
        ssassert(s.temp_heap != nullptr, "out of memory");
    }
    void *ptr = mi_heap_zalloc(s.temp_heap, size);
    ssassert(ptr != nullptr, "out of memory");
    return ptr;
}

void FreeAllTemporary() {
    Solver &s = EnsureCurrentSolver();
    if(s.temp_heap != nullptr) {
        mi_heap_destroy(s.temp_heap);
        s.temp_heap = nullptr;
    }
}

}
}
