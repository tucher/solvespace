//-----------------------------------------------------------------------------
// Solver — implementation. See solver.h for the design rationale.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "solver.h"

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
}

}  // namespace SolveSpace
