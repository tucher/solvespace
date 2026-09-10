//-----------------------------------------------------------------------------
// Solver — implementation. See solver.h for the design rationale.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "solver.h"

#include <cstdlib>

namespace SolveSpace {

// Big enough that a cached Jacobian rarely needs more than a handful of
// chunks, small enough that an unused arena costs little.
static const size_t EXPR_ARENA_CHUNK_BYTES = 64 * 1024;

ExprArena::~ExprArena() {
    for(Chunk &c : chunks) {
        std::free(c.base);
    }
}

void *ExprArena::Alloc(size_t size) {
    const size_t align = sizeof(void *);
    size = (size + align - 1) & ~(align - 1);
    if(chunks.empty() || chunks.back().used + size > chunks.back().cap) {
        size_t cap = EXPR_ARENA_CHUNK_BYTES;
        if(cap < size) cap = size;
        // calloc, so every allocation handed out is already zeroed and stays
        // that way — nothing in this arena is ever individually freed.
        char *base = static_cast<char *>(std::calloc(1, cap));
        ssassert(base != nullptr, "out of memory");
        chunks.push_back({base, 0, cap});
    }
    Chunk &c = chunks.back();
    void *p = c.base + c.used;
    c.used += size;
    return p;
}

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
    delete persistent_arena;
    persistent_arena = nullptr;
}

ExprArena *Solver::EnsurePersistentArena() {
    if(persistent_arena == nullptr) {
        persistent_arena = new ExprArena();
    }
    return persistent_arena;
}

void Solver::InvalidateJacobianCache() {
    delete persistent_arena;
    persistent_arena = nullptr;
    // The cache lives on System; flip its valid flag through the
    // back-pointer so any later `System::Solve` rebuilds from
    // scratch. (sys->jacobian_cache_valid is added in Phase 2.3.)
    sys->jacobian_cache_valid = false;
    // Drop the cached substitution map too — its Param* point into `param`,
    // which the next slow-path solve will rebuild.
    sys->cached_subMap.clear();
}

}  // namespace SolveSpace
