//-----------------------------------------------------------------------------
// Solver — encapsulates all mutable per-solve / per-sketch state.
//
// Background:
//   Historically SolveSpace's solver state lived in three process-globals:
//   `Sketch SK`, `System SYS` (file-static in slvs/lib.cpp), and
//   `dragged` (also file-static). Plus a thread-local mimalloc heap
//   (`TempArena` in platformbase.cpp). That made running multiple
//   independent `SolveSpaceEngine` instances in one process impossible.
//
//   This header introduces a `Solver` class that owns those four pieces
//   of state. Multiple `Solver` instances coexist freely, each with its
//   own sketch, system, dragged set, and scratch arena.
//
// API layering (intentional — see the design discussion):
//
//   * **Public C API** (`slvs.h`): every `Slvs_*` function takes a
//     `Slvs_Solver*` as its first argument. Fully explicit — callers
//     never rely on hidden state.
//   * **Public Python API** (Cython binding): only methods on the
//     `Solver` class are exposed; there are no module-level functions.
//     Each method passes its owner's handle to the underlying C API.
//   * **Internal C++ implementation**: each `Slvs_*` C function does an
//     atomic save → set → call → restore of a thread-local
//     `CurrentSolver*` pointer; the existing internal code (~290
//     references to `SK` / `SYS` / `dragged` across constrainteq.cpp,
//     entity.cpp, system.cpp, slvs/lib.cpp) continues to read those via
//     macros, transparently routed through the active Solver. **This
//     thread-local is purely an implementation detail invisible to
//     every caller above the C API boundary.** A future refactor will
//     thread `Solver*` through the C++ internals explicitly, removing
//     even this last vestige of implicit state. The Python and C public
//     APIs are already as bulletproof as that refactor will make them.
//
//   Each `Slvs_*` entry point is one indivisible save→set→call→restore.
//   Multi-thread: each thread's thread-local is independent. Async on a
//   single thread: each call is atomic, so awaiting Python code between
//   slvs calls never observes a stale current.

#ifndef SOLVESPACE_SOLVER_H
#define SOLVESPACE_SOLVER_H

#include "handle.h"
#include "param.h"

// Forward-declared so this header doesn't pull in <mimalloc.h>.
struct mi_heap_s;
typedef struct mi_heap_s mi_heap_t;

namespace SolveSpace {

class Sketch;
class System;

// All mutable solver state. One instance per logical sketch/solver
// session. Lifetimes: created via `Slvs_CreateSolver`, destroyed via
// `Slvs_DestroySolver`. From C++ either use the C API or construct
// directly via `new Solver()`.
class Solver {
public:
    Sketch    *sk         = nullptr;
    System    *sys        = nullptr;
    ParamSet  *dragged    = nullptr;
    mi_heap_t *temp_heap  = nullptr;

    Solver();
    ~Solver();

    Solver(const Solver&)            = delete;
    Solver& operator=(const Solver&) = delete;
};

// Thread-local pointer to the "active" Solver — set transiently by
// the C API entry points (see slvs/lib.cpp) and read by the SK / SYS
// macros below. **Implementation detail**; users above the C API do
// not interact with this. It exists to let the existing internal C++
// code (which references SK / SYS as if they were globals) keep
// working unchanged while the public API becomes fully explicit.
extern thread_local Solver *CurrentSolver;

// Returns *CurrentSolver, lazy-allocating a per-thread default if none
// is set. Mid-refactor: a few internal call sites that aren't yet
// reachable only via the explicit-Solver C API may still invoke
// macros without a Solver being explicitly current. Lazy default
// keeps those working without forcing the full mechanical refactor in
// this session. The lazy default is removed in the future-session
// follow-up that threads Solver* through the C++ internals.
inline Solver &EnsureCurrentSolver() {
    extern Solver &EnsureCurrentSolverSlow();
    if(CurrentSolver != nullptr) return *CurrentSolver;
    return EnsureCurrentSolverSlow();
}

}  // namespace SolveSpace

// Legacy-name macros so the existing ~290 `SK.foo`-style references
// across constrainteq.cpp / entity.cpp / system.cpp / expr.cpp /
// util.cpp / slvs/lib.cpp continue to work. Each access goes through
// `EnsureCurrentSolver()` (one inlined branch + a pointer deref). SK
// / SYS are 2- and 3-letter uppercase tokens with no collision risk;
// `dragged` deliberately has NO macro because it's also the name of
// a field on `Slvs_System` and `System` (see solver.h commit notes).
#define SK   (*(::SolveSpace::EnsureCurrentSolver().sk))
#define SYS  (*(::SolveSpace::EnsureCurrentSolver().sys))

#endif  // SOLVESPACE_SOLVER_H
