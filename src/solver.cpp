//-----------------------------------------------------------------------------
// Solver — implementation. See solver.h for the design rationale.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "solver.h"

#include <mimalloc.h>

namespace SolveSpace {

thread_local Solver *CurrentSolver = nullptr;

// Per-thread default Solver, allocated on first access by the
// EnsureCurrentSolver lazy path. RAII storage ensures the Solver is
// destroyed when the thread exits — `thread_local` member destruction
// is the standard mechanism for thread-local heap ownership.
namespace {
struct DefaultSolverStorage {
    Solver *ptr = nullptr;
    ~DefaultSolverStorage() {
        delete ptr;
    }
};
thread_local DefaultSolverStorage default_solver;
}  // namespace

Solver &EnsureCurrentSolverSlow() {
    if(default_solver.ptr == nullptr) {
        default_solver.ptr = new Solver();
    }
    CurrentSolver = default_solver.ptr;
    return *CurrentSolver;
}

Solver::Solver() {
    sk      = new Sketch();
    sys     = new System();
    dragged = new ParamSet();
}

Solver::~Solver() {
    // Destroy in reverse construction order. mimalloc heap last so any
    // Expr/AllocTemporary holdouts in Sketch / System are still valid
    // during their destructors (defensive — should be empty in practice).
    delete dragged;
    delete sys;
    delete sk;
    if(temp_heap != nullptr) {
        mi_heap_destroy(temp_heap);
        temp_heap = nullptr;
    }
}

}  // namespace SolveSpace
