//-----------------------------------------------------------------------------
// Solver — implementation. See solver.h for the design rationale.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "solver.h"

#include <mimalloc.h>

namespace SolveSpace {

Solver::Solver() {
    sk      = new Sketch();
    sys     = new System();
    dragged = new ParamSet();
    // Back-pointers — Sketch::owner / System::owner let methods on
    // those types reach the owning Solver without any thread-local.
    sk->owner  = this;
    sys->owner = this;
}

Solver::~Solver() {
    delete dragged;
    delete sys;
    delete sk;
    if(persistent_heap != nullptr) {
        mi_heap_destroy(persistent_heap);
        persistent_heap = nullptr;
    }
}

mi_heap_t *Solver::EnsurePersistentHeap() {
    if(persistent_heap == nullptr) {
        persistent_heap = mi_heap_new();
        ssassert(persistent_heap != nullptr, "out of memory");
    }
    return persistent_heap;
}

void Solver::InvalidateJacobianCache() {
    if(persistent_heap != nullptr) {
        mi_heap_destroy(persistent_heap);
        persistent_heap = nullptr;
    }
    // The cache lives on System; flip its valid flag through the
    // back-pointer so any later `System::Solve` rebuilds from
    // scratch. (sys->jacobian_cache_valid is added in Phase 2.3.)
    sys->jacobian_cache_valid = false;
}

}  // namespace SolveSpace
