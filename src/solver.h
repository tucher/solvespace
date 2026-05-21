//-----------------------------------------------------------------------------
// Solver — owns all per-sketch / per-solve state.
//
// Architecture (post-Phase-0 mechanical refactor):
//
//   Every piece of solver state — sketch, system, dragged-param set —
//   lives on a `Solver` instance. Multiple `Solver`s coexist freely.
//   No globals; no thread-locals for state. The temporary scratch
//   arena (used by `AllocExpr`) is a separate thread-local **workspace**
//   in platformbase.cpp — it is per-thread because solves never overlap
//   on one thread, not per-Solver, so two Solvers on the same thread
//   harmlessly share the arena across consecutive solves.
//
// API layering:
//
//   * **Public C API** (`slvs.h`): every data-mutating `Slvs_*` function
//     takes a `Slvs_Solver *` as its first argument.
//   * **Public Python API** (Cython binding): only methods on the
//     `Solver` class are exposed.
//   * **Internal C++**: each `Slvs_*` entry point casts the opaque
//     handle to `SolveSpace::Solver *solver_cpp` and the implementation
//     reaches `Sketch`/`System` directly through that pointer or via
//     the back-pointers `EntityBase::sk` / `ConstraintBase::sk` /
//     `Param::sk` (set during insertion) and `System::owner` /
//     `Sketch::owner`. There is no thread-local indirection — the
//     active Solver is named explicitly at every site that uses it.

#ifndef SOLVESPACE_SOLVER_H
#define SOLVESPACE_SOLVER_H

#include "handle.h"
#include "param.h"

namespace SolveSpace {

class Sketch;
class System;

// Owns all per-sketch / per-solve state. Lifetimes: created via
// `Slvs_CreateSolver` (or `new Solver()` in C++), destroyed via
// `Slvs_DestroySolver` (or `delete`).
class Solver {
public:
    Sketch    *sk      = nullptr;
    System    *sys     = nullptr;
    ParamSet  *dragged = nullptr;

    // When true, `Slvs_SolveSketch` skips the post-solve rank test
    // (which costs ~13% wall-clock on a typical delta-style sketch).
    // Trade-off: the solver returns `OKAY` instead of `REDUNDANT_OKAY`
    // for over-constrained-but-consistent systems, and `dof` is not
    // reported. The numeric solution is identical either way.
    //
    // pyactiongraphsim's planner validates topology at build time so
    // redundancy is impossible by construction; the engine sets this
    // to `true` after `engine.build()`. Default false for callers
    // that need the diagnostic.
    bool      suppress_rank_test = false;

    Solver();
    ~Solver();

    Solver(const Solver&)            = delete;
    Solver& operator=(const Solver&) = delete;
};

}  // namespace SolveSpace

#endif  // SOLVESPACE_SOLVER_H
