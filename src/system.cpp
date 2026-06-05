//-----------------------------------------------------------------------------
// Once we've written our constraint equations in the symbolic algebra system,
// these routines linearize them, and solve by a modified Newton's method.
// This also contains the routines to detect non-convergence or inconsistency,
// and report diagnostics to the user.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------
#include "solvespace.h"

#include <Eigen/Core>
#include <Eigen/SparseQR>
#include <Eigen/SparseCholesky>
#include <Eigen/OrderingMethods>

#include <cstdlib>

namespace SolveSpace {

// The solver will converge all unknowns to within this tolerance. This must
// always be much less than LENGTH_EPS, and in practice should be much less.
const double System::CONVERGE_TOLERANCE = (LENGTH_EPS/(1e2));

constexpr size_t LikelyPartialCountPerEq = 10;

// ── Sketch insertion helpers ────────────────────────────────────────
//
// Defined out-of-line because they call `Solver::InvalidateJacobianCache`,
// whose declaration arrives after Sketch in the header include order.
// Every topology insertion drops the cached symbolic Jacobian — the
// new entity/constraint/param could appear in equations the cached
// `mat.A.sym` doesn't reflect.

#define INVALIDATE_CACHE_IF_OWNED() do { \
    if(this->owner != nullptr) this->owner->InvalidateJacobianCache(); \
} while(0)

hEntity Sketch::AddEntity(EntityBase *e) {
    INVALIDATE_CACHE_IF_OWNED();
    hEntity h = entity.AddAndAssignId(e);
    entity.FindById(h)->sk = this;
    return h;
}
hConstraint Sketch::AddConstraint(ConstraintBase *c) {
    INVALIDATE_CACHE_IF_OWNED();
    hConstraint h = constraint.AddAndAssignId(c);
    constraint.FindById(h)->sk = this;
    return h;
}
hParam Sketch::AddParam(Param *p) {
    INVALIDATE_CACHE_IF_OWNED();
    hParam h = param.AddAndAssignId(p);
    param.FindById(h)->sk = this;
    return h;
}
void Sketch::AddEntityKeepingHandle(EntityBase *e) {
    INVALIDATE_CACHE_IF_OWNED();
    entity.Add(e);
    entity.FindById(e->h)->sk = this;
}
void Sketch::AddConstraintKeepingHandle(ConstraintBase *c) {
    INVALIDATE_CACHE_IF_OWNED();
    constraint.Add(c);
    constraint.FindById(c->h)->sk = this;
}
void Sketch::AddParamKeepingHandle(Param *p) {
    INVALIDATE_CACHE_IF_OWNED();
    param.Add(p);
    param.FindById(p->h)->sk = this;
}

#undef INVALIDATE_CACHE_IF_OWNED

// Walk `mat.A.sym` and `mat.B.sym` and deep-copy every Expr tree into
// `solver->persistent_heap`, replacing the pointers in-place. After
// this call, none of the cached Exprs reference the per-solve
// TempArena that `Slvs_SolveSketch` is about to free; the cache
// survives across solves until invalidated.
//
// Caller invariant: `mat.A.sym` and `mat.B.sym` were just populated
// by `WriteJacobian`. The Exprs are still in the temp arena at
// promotion time (DeepCopyIntoHeap reads them); we replace each
// pointer with the persistent copy.
static void PromoteJacobianToPersistent(System *sys) {
    mi_heap_t *heap = sys->owner->EnsurePersistentHeap();
    using namespace Eigen;
    const int outer = sys->mat.A.sym.outerSize();
    for(int k = 0; k < outer; k++) {
        for(SparseMatrix<Expr *>::InnerIterator it(sys->mat.A.sym, k); it; ++it) {
            it.valueRef() = it.value()->DeepCopyIntoHeap(heap);
        }
    }
    for(size_t i = 0; i < sys->mat.B.sym.size(); i++) {
        sys->mat.B.sym[i] = sys->mat.B.sym[i]->DeepCopyIntoHeap(heap);
    }
}

bool System::WriteJacobian(int tag) {
    // Clear all
    mat.param.clear();
    mat.eq.clear();
    mat.A.sym.setZero();
    mat.B.sym.clear();

    for(Equation &e : eq) {
        if(e.tag != tag) continue;
        mat.eq.push_back(&e);
    }
    if(mat.eq.size() >= MAX_UNKNOWNS) {
        return false;
    }
    mat.m = mat.eq.size();

    std::unordered_map<uint32_t, int> paramToIndex;
    for(Param &p : param) {
        if(p.tag != tag) continue;
        // Fill the param id to index map
        paramToIndex[p.h.v] = mat.param.size();
        mat.param.push_back(p.h);
    }
    mat.n = mat.param.size();

    // In some experimenting, this is almost always the right size.
    // Value is usually between 0 and 20, comes from number of constraints?
    mat.A.sym.resize(mat.m, mat.n);
    mat.A.sym.reserve(Eigen::VectorXi::Constant(mat.n, LikelyPartialCountPerEq));

    mat.B.sym.reserve(mat.eq.size());
    for(size_t i = 0; i < mat.eq.size(); i++) {
        Equation *e = mat.eq[i];
        // Deep-copy and simplify (fold) the current equation.
        Expr *f = e->e->DeepCopyWithParamsAsPointers(&param, &(owner->sk->param), /*foldConstants=*/true);

        ParamSet paramsUsed;
        f->ParamsUsedList(&paramsUsed);

        for(hParam p : paramsUsed) {
            // Find the index of this parameter
            auto it = paramToIndex.find(p.v);
            if(it == paramToIndex.end()) continue;
            // this is the parameter index
            const int j = it->second;
            // compute partial derivative of f
            Expr *pd = f->PartialWrt(p);
            pd = pd->FoldConstants(/*allocCopy=*/false);
            if(pd->IsZeroConst())
                continue;
            mat.A.sym.insert(i, j) = pd;
        }
        mat.B.sym.push_back(f);
    }
    return true;
}

void System::EvalJacobian() {
    using namespace Eigen;
    mat.A.num.setZero();
    mat.A.num.resize(mat.m, mat.n);
    const int size = mat.A.sym.outerSize();

    for(int k = 0; k < size; k++) {
        for(SparseMatrix <Expr *>::InnerIterator it(mat.A.sym, k); it; ++it) {
            double value = it.value()->Eval();
            if(EXACT(value == 0.0)) continue;
            mat.A.num.insert(it.row(), it.col()) = value;
        }
    }
    mat.A.num.makeCompressed();
}

bool System::IsDragged(hParam p) {
    return dragged.find(p) != dragged.end();
}

SubstitutionMap System::SolveBySubstitution() {
    // Contains pointers to last substitutions in a substitution chain
    std::vector<Param *> subVec;
    // Maps a parameter to the index of its last substitution in  `subVec`
    std::unordered_map<hParam, size_t, HandleHasher<hParam>> leaves;
    // Tracks how many slots in `subVec` contain a specific last substitution
    // (this can happen as slots that once contained a specific substitution
    //  are updated to point to another over the run of the substitution algorithm)
    std::unordered_map<hParam, std::vector<size_t>, HandleHasher<hParam>> slotTrack;

    for(auto &teq : eq) {
        Expr *tex = teq.e;

        // If we have `(a - b) = 0` where both a and b are parameters, then `a = b` and we can substitute
        if(tex->op    == Expr::Op::MINUS &&
           tex->a->op == Expr::Op::PARAM &&
           tex->b->op == Expr::Op::PARAM)
        {
            Param *sub = param.FindByIdNoOops(tex->a->parh);
            Param *by = param.FindByIdNoOops(tex->b->parh);
            if(!sub || !by) {
                // Don't substitute unless they're both solver params;
                // otherwise it's an equation that can be solved immediately,
                // or an error to flag later.
                continue;
            }

            if(sub->h == by->h) {
                teq.tag = EQ_SUBSTITUTED;
                continue;
            }

            // Take the last substitution of parameter a
            size_t subIdx = 0;
            auto it = leaves.find(sub->h);
            if(it != leaves.end()) {
                subIdx = it->second;
                sub = subVec.at(it->second - 1);
            }

            // Take the last substitution of parameter b
            size_t byIdx = 0;
            it = leaves.find(by->h);
            if(it != leaves.end()) {
                byIdx = it->second;
                by = subVec.at(it->second - 1);
            }

            // If the last substituton of `a` is a dragged param, keep it
            // and substitute the other param
            if(IsDragged(sub->h)) {
                std::swap(sub, by);
                std::swap(subIdx, byIdx);
            }

            if(subIdx == 0) {
                if(byIdx == 0) {
                    // Neither `sub` nor `by` are in the map, so add them and
                    // set the target index
                    subVec.push_back(by);
                    leaves[by->h] = leaves[sub->h] = subVec.size();
                } else {
                    // `sub` isn't in the map, but `by` is, so just add `sub` to
                    // the map with `by` as the target
                    leaves[sub->h] = byIdx;
                }
            } else {
                // `sub` already exists in the map, so just update any slots
                // that point to it as the last substitution to point to `by`
                // instead
                auto it = slotTrack.find(sub->h);
                if(it == slotTrack.end()) {
                    // There's only this one slot, so just replace it with `by`
                    subVec[subIdx - 1] = by;

                    // If `by` was already in the map, that means we now have
                    // an additional slot where it resides, so add it to the
                    // slot tracker
                    if(byIdx != 0) {
                        // If `by` is already in the slot tracker, we'll get
                        // back a vector with at least two elements; otherwise
                        // this access will add a new item to the slot tracker
                        // with an empty vector
                        auto &bySlots = slotTrack[by->h];
                        if(bySlots.empty()) {
                            bySlots.push_back(byIdx);
                        }
                        bySlots.push_back(subIdx);
                    }
                } else {
                    // We have more than one slot pointing to `sub`, so update
                    // all of the slots to point to `by`
                    for(size_t i : it->second) {
                        subVec[i - 1] = by;
                    }

                    // No more slots are pointing to `sub`, so extract the slot list
                    // and erase `sub` from the tracker
                    auto subSlots = std::move(it->second);
                    slotTrack.erase(it);

                    // Same as above: this access either gives us an existing vector
                    // with at least two elements, or creates an empty vector
                    auto &bySlots = slotTrack[by->h];
                    if(bySlots.empty()) {
                        bySlots = std::move(subSlots);
                        if(byIdx != 0) {
                            bySlots.push_back(byIdx);
                        }
                    } else {
                        bySlots.insert(bySlots.end(), subSlots.begin(), subSlots.end());
                    }
                }

                leaves[by->h] = subIdx;
            }

            sub->tag = VAR_SUBSTITUTED;
            teq.tag = EQ_SUBSTITUTED;
        }
    }

    SubstitutionMap subMap;
    for(auto &sub : leaves) {
        Param *by = subVec[sub.second - 1];
        if(sub.first != by->h) {
            subMap[sub.first] = by;
        }
    }

    // Substitute all the equations
    for(auto &req : eq) {
        req.e->Substitute(subMap);
    }

    return subMap;
}

//-----------------------------------------------------------------------------
// Calculate the rank of the Jacobian matrix
//-----------------------------------------------------------------------------
int System::CalculateRank() {
    using namespace Eigen;
    if(mat.n == 0 || mat.m == 0) return 0;
    // NOTE: this is a rank-revealing SparseQR on the full Jacobian A — a
    // SECOND factorization, and (unlike the LDLT step solve) it is still
    // superlinear (~O(n^2.5)) because Eigen's SparseQR does not exploit block
    // structure. It runs only when the rank test is enabled
    // (suppress_rank_test=False). Production multi-robot scenes should build
    // the engine with suppress_rank_test=True (the showroom does); the rank
    // test is a diagnostic, not a per-tick necessity. Making this near-linear
    // (rank from the step's LDLT pivots) is deferred — see the upgrade plan.
    SparseQR <SparseMatrix<double>, COLAMDOrdering<int>> solver;
    solver.compute(mat.A.num);
    int result = solver.rank();
    return result;
}

bool System::TestRank(int *dof, int *rank) {
    EvalJacobian();
    int jacobianRank = CalculateRank();
    // We are calculating dof based on real rank, not mat.m.
    // Using this approach we can calculate real dof even when redundant is allowed.
    if(dof != NULL) *dof = mat.n - jacobianRank;
    if(rank) {
        *rank = jacobianRank;
    }
    return jacobianRank == mat.m;
}

// pImpl placeholder — kept as a struct so the explicit `~System()`
// in `solvespace.h` still references a complete type. Phase 1.3's
// SparseQR analyzePattern cache lived here and was removed because
// Eigen's `SparseQR::analyzePattern` retains references into the
// analyzed matrix's index/value arrays, but `AAt = mat.A.num * ...`
// reallocates those arrays on every Newton iteration. The pattern
// cache dangled and crashed intermittently in
// test_slvs_incremental_delta (the test that exercises the rank-
// test path that Phase 1.2 lets the engine opt out of). The ~2%
// gain wasn't worth the soundness hazard; the bigger Phase 2
// symbolic-Jacobian cache (this file's `jacobian_cache_valid`
// path) doesn't depend on it.
struct System::LinearSolverCache {};

bool System::SolveLinearSystem(const Eigen::SparseMatrix <double> &A,
                               const Eigen::VectorXd &B, Eigen::VectorXd *X)
{
    if(A.outerSize() == 0) return true;
    using namespace Eigen;
    // `A` is the SPD (first-kind) normal matrix Aᵀ·A + λI from
    // SolveLeastSquares — strictly positive-definite by the ridge term, so a
    // fill-reducing sparse LDLT (AMD ordering) factors it. For a
    // block-diagonal / bounded-treewidth system this is near-linear, whereas
    // Eigen's SparseQR on the old A·Aᵀ was empirically ~O(n^2.5) — it did not
    // exploit the block structure (measured: ~95% of solve time, ~5.7× per
    // size-doubling on disconnected robots). On the rare event LDLT reports
    // failure (NaN/Inf in the Jacobian) we return false and let the caller's
    // designed hard-fail path handle it (Newton non-convergence →
    // DIDNT_CONVERGE / engine rebuild) — no silent fallback masking a fault.
    SimplicialLDLT<SparseMatrix<double>, Lower, AMDOrdering<int>> ldlt;
    ldlt.compute(A);
    if(ldlt.info() != Success) return false;
    *X = ldlt.solve(B);
    // Derive the column-nullity (system DOF) from the pivots of
    // A = normalMat = AᵀA + λI. A direction in null(AᵀA) shows up as a
    // pivot ≈ λ (1e-9), whereas a genuinely-constrained direction's pivot
    // is the corresponding eigenvalue of AᵀA — orders of magnitude larger
    // (empirically ≥ ~0.1 vs the ~2e-9 null floor: an ~8-decade gap). The
    // 1e-6 cutoff sits in that gap; misclassifying a (near-singular) real
    // pivot as null only forgoes a cache entry (safe), never the reverse.
    // The caller uses this to refuse caching a non-unique system.
    {
        const double null_pivot_cutoff = 1e-6;
        Eigen::VectorXd D = ldlt.vectorD();
        int nullity = 0;
        for(int i = 0; i < D.size(); i++) {
            if(fabs(D[i]) < null_pivot_cutoff) nullity++;
        }
        last_jacobian_nullity = nullity;
    }
    return (ldlt.info() == Success);
}

bool System::SolveLeastSquares() {
    using namespace Eigen;
    // Scale the columns; this scale weights the parameters for the least
    // squares solve, so that we can encourage the solver to make bigger
    // changes in some parameters, and smaller in others.
    VectorXd scale = VectorXd::Ones(mat.n);
    for(int c = 0; c < mat.n; c++) {
        if(IsDragged(mat.param[c])) {
            // It's least squares, so this parameter doesn't need to be all
            // that big to get a large effect.
            scale[c] = 1 / 20.0;
        }
    }

    const int size = mat.A.num.outerSize();
    for(int k = 0; k < size; k++) {
        for(SparseMatrix<double>::InnerIterator it(mat.A.num, k); it; ++it) {
            it.valueRef() *= scale[it.col()];
        }
    }

    // First-kind normal equations (Levenberg–Marquardt / ridge step):
    //   (AᵀA + λI) X = Aᵀ B,   X then unscaled by the column weights.
    // We deliberately do NOT use the second-kind form (z=(AAᵀ)⁻¹B; X=Aᵀz):
    // the tetra body parametrization is inherently rank-deficient (12 params,
    // 6 gauge DOF per body), so AAᵀ is singular and the second-kind null-space
    // components of z blow up as 1/λ, leaving a cancellation-noise floor in
    // X=Aᵀz that stalls Newton below CONVERGE_TOLERANCE. The first-kind form
    // regularizes X directly and is amplification-free: AᵀB ∈ range(Aᵀ) ⊥
    // null(AᵀA), so even a tiny λ injects no null-space noise. As λ→0 it
    // equals the prior pseudoinverse step (precision preserved); λ small and
    // fixed keeps it ≈ Gauss-Newton. AᵀA is SPD and block-diagonal-preserving,
    // so the LDLT+AMD factorization stays near-linear.
    const double lambda = 1e-9;
    VectorXd Atb = mat.A.num.transpose() * mat.B.num;
    SparseMatrix<double> I(mat.n, mat.n);
    I.setIdentity();
    normalMat = mat.A.num.transpose() * mat.A.num + lambda * I;
    normalMat.makeCompressed();

    if(!SolveLinearSystem(normalMat, Atb, &mat.X)) return false;

    for(int c = 0; c < mat.n; c++) {
        mat.X[c] *= scale[c];
    }
    return true;
}

bool System::NewtonSolve() {
    int iter = 0;
    bool converged = false;
    int i;

    // Evaluate the functions at our operating point.
    mat.B.num = Eigen::VectorXd(mat.m);
    for(i = 0; i < mat.m; i++) {
        mat.B.num[i] = (mat.B.sym[i])->Eval();
    }
    do {
        // And evaluate the Jacobian at our initial operating point.
        EvalJacobian();

        if(!SolveLeastSquares()) break;

        // Take the Newton step;
        //      J(x_n) (x_{n+1} - x_n) = 0 - F(x_n)
        for(i = 0; i < mat.n; i++) {
            Param *p = param.FindById(mat.param[i]);
            p->val -= mat.X[i];
            if(IsReasonable(p->val)) {
                // Very bad, and clearly not convergent
                last_newton_iterations = iter + 1;
                return false;
            }
        }

        // Re-evalute the functions, since the params have just changed.
        for(i = 0; i < mat.m; i++) {
            mat.B.num[i] = (mat.B.sym[i])->Eval();
            if(IsReasonable(mat.B.num[i])) {
                // Very bad, and clearly not convergent
                last_newton_iterations = iter + 1;
                return false;
            }
        }

        // Check for convergence
        converged = true;
        for(i = 0; i < mat.m; i++) {
            if(fabs(mat.B.num[i]) > CONVERGE_TOLERANCE) {
                converged = false;
                break;
            }
        }
    } while(iter++ < 50 && !converged);

    last_newton_iterations = iter;
    return converged;
}

void System::WriteEquationsExceptFor(hConstraint hc, Group *g) {
    // Generate all the equations from constraints in this group
    for(auto &con : owner->sk->constraint) {
        ConstraintBase *c = &con;
        if(c->group != g->h) continue;
        if(c->h == hc) continue;

        if(c->HasLabel() && c->type != Constraint::Type::COMMENT &&
                g->allDimsReference)
        {
            // When all dimensions are reference, we adjust them to display
            // the correct value, and then don't generate any equations.
            c->ModifyToSatisfy();
            continue;
        }
        if(g->relaxConstraints && c->type != Constraint::Type::POINTS_COINCIDENT) {
            // When the constraints are relaxed, we keep only the point-
            // coincident constraints, and the constraints generated by
            // the entities and groups.
            continue;
        }

        c->GenerateEquations(&eq);
    }
    // And the equations from entities
    for(auto &ent : owner->sk->entity) {
        EntityBase *e = &ent;
        if(e->group != g->h) continue;

        e->GenerateEquations(&eq);
    }
    // And from the groups themselves
    g->GenerateEquations(&eq);
}

void System::FindWhichToRemoveToFixJacobian(Group *g, List<hConstraint> *bad, bool forceDofCheck) {
    auto time = GetMilliseconds();
    g->solved.timeout = false;
    int a;

    for(a = 0; a < 2; a++) {
        for(auto &con : owner->sk->constraint) {
            if((GetMilliseconds() - time) > g->solved.findToFixTimeout) {
                g->solved.timeout = true;
                return;
            }

            ConstraintBase *c = &con;
            if(c->group != g->h) continue;
            if((c->type == Constraint::Type::POINTS_COINCIDENT && a == 0) ||
               (c->type != Constraint::Type::POINTS_COINCIDENT && a == 1))
            {
                // Do the constraints in two passes: first everything but
                // the point-coincident constraints, then only those
                // constraints (so they appear last in the list).
                continue;
            }

            param.ClearTags();
            eq.Clear();
            WriteEquationsExceptFor(c->h, g);
            eq.ClearTags();

            // It's a major speedup to solve the easy ones by substitution here,
            // and that doesn't break anything.
            if(!forceDofCheck) {
                SolveBySubstitution();
            }

            WriteJacobian(0);
            EvalJacobian();

            int rank = CalculateRank();
            if(rank == mat.m) {
                // We fixed it by removing this constraint
                bad->Add(&(c->h));
            }
        }
    }
}

SolveResult System::Solve(Group *g, int *dof, List<hConstraint> *bad,
                          bool andFindBad, bool andFindFree, bool forceDofCheck)
{
    // Hoisted so the early `goto didnt_converge`s in both branches
    // don't jump across a local's initialisation (ill-formed for any
    // non-trivially-initialised local — `SubstitutionMap` is a
    // `std::unordered_map` with a non-trivial default ctor).
    bool rankOk;
    SubstitutionMap subMap;
    // ── Phase 2 cache-hit fast path ─────────────────────────────────
    //
    // When `jacobian_cache_valid` is set by a previous successful
    // solve, `mat.A.sym` and `mat.B.sym` are still populated with the
    // symbolic Jacobian and residuals — their Expr trees live in the
    // per-Solver persistent heap and reference PARAM_PTRs into
    // `param` (whose storage we kept stable) and CONST_PTRs into
    // `ConstraintBase::valA` (also stable for the constraint's
    // lifetime). Slvs_SolveSketch refreshed `param[i].val` from the
    // sketch before calling us; we skip everything up to NewtonSolve.
    //
    // Caller invariants enforced upstream (slvs/lib.cpp):
    //   - No mutating Slvs_* call has fired since the last solve.
    //   - `dragged` was re-applied.
    //   - `param[i].val` was refreshed from `sk->param[i].val`.
    //
    // If any of those is wrong the cache should already have been
    // invalidated by `Solver::InvalidateJacobianCache`.
    if(jacobian_cache_valid) {
        if(dof != NULL) *dof = -1;
        if(!NewtonSolve()) {
            // Don't drop the cache — the topology hasn't changed; the
            // user can adjust inputs and try again. didnt_converge
            // is handled by the same diagnostic path as the slow path.
            rankOk = true;
            goto didnt_converge;
        }
        // A system that was unique at cache-build can move into a
        // non-unique configuration (a kinematic singularity opening up a
        // free DOF). NewtonSolve just refreshed `last_jacobian_nullity`
        // from the LDLT pivots; if it's no longer 0 the cached path can't
        // be trusted for subsequent ticks — drop the cache so the next
        // solve rebuilds and re-evaluates cacheability.
        if(last_jacobian_nullity != 0) {
            owner->InvalidateJacobianCache();
        }
        rankOk = (!g->suppressDofCalculation) ? TestRank(dof) : true;
        if(!rankOk) {
            if(andFindBad) {
                // FindWhichToRemoveToFixJacobian rebuilds mat from
                // scratch — invalidate so the next solve starts fresh.
                owner->InvalidateJacobianCache();
                FindWhichToRemoveToFixJacobian(g, bad, forceDofCheck);
            }
        } else {
            MarkParamsFree(andFindFree);
        }
        for(auto &p : param) {
            Param *pp = owner->sk->GetParam(p.h);
            pp->val   = p.val;
            pp->known = true;
            pp->free  = p.free;
        }
        // Substituted params (folded out by SolveBySubstitution at cache
        // build) are absent from the solved system, so the loop above wrote
        // them stale. Set each to its substitution target's freshly-solved
        // value — exactly what the slow-path write-back does via `subMap`.
        // Without this they FREEZE at the build pose on every cache hit: e.g.
        // a serial DH arm's coincident joint points stay put, so intermediate
        // links are broken while the end-effector (built from solved params)
        // looks correct. (`cached_subMap`'s Param* point into `param`, kept
        // stable for the cache's lifetime; cleared on InvalidateJacobianCache.)
        for(auto &kv : cached_subMap) {
            owner->sk->GetParam(kv.first)->val = kv.second->val;
        }
        return rankOk ? SolveResult::OKAY : SolveResult::REDUNDANT_OKAY;
    }
    // ── Slow path: build mat from scratch ──────────────────────────

    WriteEquationsExceptFor(Constraint::NO_CONSTRAINT, g);

    // int x;
    // printf("%d equations", eq.n);
    // for(x = 0; x < eq.n; x++) {
    //     printf("  %.3f = %s = 0", eq[x].e->Eval(), eq[x].e->Print().c_str());
    // }
    // printf("%d parameters", param.n);
    // for(x = 0; x < param.n; x++) {
    //     printf("   param %08x at %.3f", param[x].h.v, param[x].val);
    // }

    // All params and equations are assigned to group zero.
    param.ClearTags();
    eq.ClearTags();

    // Since we are suppressing dof calculation or allowing redundant, we
    // can't / don't want to catch result of dof checking without substitution
    if(g->suppressDofCalculation || g->allowRedundant || !forceDofCheck) {
        subMap = SolveBySubstitution();
    }

    // Write the Jacobian for the whole (post-substitution) system and do a
    // rank test; that tells us if the system is inconsistently constrained.
    //
    // NOTE: we intentionally do NOT peel "soluble-alone" single-parameter
    // equations into separate 1×1 Newton solves anymore. That upstream
    // optimisation predated the SimplicialLDLT+AMD linear solve
    // (`SolveLeastSquares` / `SolveLinearSystem`): single-param equations are
    // now trivial sparse rows that cost ~nothing in the block-diagonal big
    // solve, so peeling them buys nothing — and it forced
    // `jacobian_cache_valid` off for any scene built from independent
    // mechanisms (the cache only ever covered the tag-0 remainder). Folding
    // them into the one big system makes those scenes cacheable like any
    // coupled mechanism. See the symbolic-Jacobian cache history for context.
    if(!WriteJacobian(0)) {
        return SolveResult::TOO_MANY_UNKNOWNS;
    }
    // The actual promote-to-persistent is deferred until after NewtonSolve,
    // once `last_jacobian_nullity` is known — see below. Promoting here would
    // deep-copy the symbolic Jacobian every tick for non-unique (nullity>0)
    // systems whose cache is then refused.
    // Clear dof value in order to have indication when dof is actually not calculated
    if(dof != NULL) *dof = -1;
    // We are suppressing or allowing redundant, so we no need to catch unsolveable + redundant
    rankOk = (!g->suppressDofCalculation && !g->allowRedundant) ? TestRank(dof) : true;

    // Solve the whole system.
    if(!NewtonSolve()) {
        goto didnt_converge;
    }

    // Here we are want to calculate dof even when redundant is allowed, so just handle suppressing
    rankOk = (!g->suppressDofCalculation) ? TestRank(dof) : true;
    if(!rankOk) {
        if(andFindBad) FindWhichToRemoveToFixJacobian(g, bad, forceDofCheck);
    } else {
        MarkParamsFree(andFindFree);
    }
    // System solved correctly, so write the new values back in to the
    // main parameter table.
    for(auto &p : param) {
        auto it = subMap.find(p.h);
        double val = it == subMap.end() ? p.val : it->second->val;

        Param *pp = owner->sk->GetParam(p.h);
        pp->val = val;
        pp->known = true;
        pp->free  = p.free;
    }
    // Cache is valid for the NEXT solve only if every cache invariant
    // we set up still holds and no helper down the line rebuilt mat.
    // `FindWhichToRemoveToFixJacobian` (called for !rankOk + andFindBad
    // above) rewrites mat to its own ad-hoc form, so don't cache in
    // that case — it invalidated already.
    // Cache the symbolic Jacobian only when the solution is UNIQUE
    // (`last_jacobian_nullity == 0`). A non-unique system (free DOF) must
    // not be cached: reusing the frozen Jacobian while inputs move lets
    // the min-norm step drift onto a different constraint-satisfying
    // branch, silently corrupting otherwise well-determined sub-systems
    // (e.g. an arm sharing the solve group with an under-constrained
    // body). This is enforced independently of `rankOk` so it holds even
    // under suppress_rank_test (where rankOk is forced true).
    jacobian_cache_valid = rankOk && (last_jacobian_nullity == 0);
    if(jacobian_cache_valid) {
        // Deep-copy the symbolic Jacobian into the persistent heap so it
        // survives the per-solve temp-arena free and the next tick can
        // enter via the cache-hit fast path. Done here (not before
        // NewtonSolve) so non-unique systems — whose cache is refused —
        // don't pay the copy every tick.
        PromoteJacobianToPersistent(this);
        // Cache the substitution map so the cache-hit fast path can rebuild
        // the folded-out (substituted) params' values each tick — they're
        // not in `mat` and would otherwise freeze at this build pose. Param*
        // values point into `param`, kept stable for the cache's lifetime.
        cached_subMap = subMap;
    }
    return rankOk ? SolveResult::OKAY : SolveResult::REDUNDANT_OKAY;

didnt_converge:
    owner->sk->constraint.ClearTags();
    // Not using range-for here because index is used in additional ways
    for(size_t i = 0; i < mat.eq.size(); i++) {
        if(fabs(mat.B.num[i]) > CONVERGE_TOLERANCE || IsReasonable(mat.B.num[i])) {
            // This constraint is unsatisfied.
            if(!mat.eq[i]->h.isFromConstraint()) continue;

            hConstraint hc = mat.eq[i]->h.constraint();
            ConstraintBase *c = owner->sk->constraint.FindByIdNoOops(hc);
            if(!c) continue;
            // Don't double-show constraints that generated multiple
            // unsatisfied equations
            if(!c->tag) {
                bad->Add(&(c->h));
                c->tag = 1;
            }
        }
    }

    return rankOk ? SolveResult::DIDNT_CONVERGE : SolveResult::REDUNDANT_DIDNT_CONVERGE;
}

SolveResult System::SolveRank(Group *g, int *rank, int *dof, List<hConstraint> *bad,
                              bool andFindBad, bool andFindFree)
{
    WriteEquationsExceptFor(Constraint::NO_CONSTRAINT, g);

    // All params and equations are assigned to group zero.
    param.ClearTags();
    eq.ClearTags();

    // Now write the Jacobian, and do a rank test; that
    // tells us if the system is inconsistently constrained.
    if(!WriteJacobian(0)) {
        return SolveResult::TOO_MANY_UNKNOWNS;
    }

    bool rankOk = TestRank(dof, rank);
    if(!rankOk) {
        // When we are testing with redundant allowed, we don't want to have additional info
        // about redundants since this test is working only for single redundant constraint
        if(!g->suppressDofCalculation && !g->allowRedundant) {
            if(andFindBad) FindWhichToRemoveToFixJacobian(g, bad, true);
        }
    } else {
        MarkParamsFree(andFindFree);
    }
    return rankOk ? SolveResult::OKAY : SolveResult::REDUNDANT_OKAY;
}

void System::Clear() {
    entity.Clear();
    param.Clear();
    eq.Clear();
    dragged.clear();
    mat.A.num.setZero();
    mat.A.sym.setZero();
    // Drop the SparseQR analyzePattern cache — the next solve will
    // operate on a different (possibly differently-shaped) system.
    delete linear_solver_cache;
    linear_solver_cache = nullptr;
}

// Defined here, where `LinearSolverCache` is a complete type, so
// `delete` on the pImpl pointer compiles.
System::~System() {
    delete linear_solver_cache;
}

void System::MarkParamsFree(bool find) {
    // If requested, find all the free (unbound) variables. This might be
    // more than the number of degrees of freedom. Don't always do this,
    // because the display would get annoying and it's slow.
    for(auto &p : param) {
        p.free = false;

        if(find) {
            if(p.tag == 0) {
                p.tag = VAR_DOF_TEST;
                WriteJacobian(0);
                EvalJacobian();
                int rank = CalculateRank();
                if(rank == mat.m) {
                    p.free = true;
                }
                p.tag = 0;
            }
        }
    }
}

} // namespace SolveSpace
