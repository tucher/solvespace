//-----------------------------------------------------------------------------
// An expression in our symbolic algebra system, used to write, linearize,
// and solve our constraint equations.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------
#ifndef SOLVESPACE_EXPR_H
#define SOLVESPACE_EXPR_H

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>

#include "dsc.h"
#include "param.h"

// Forward-declared so this header doesn't pull in <mimalloc.h>.
struct mi_heap_s;
typedef struct mi_heap_s mi_heap_t;

namespace SolveSpace {

class Sketch;

using SubstitutionMap = std::unordered_map<hParam, Param *, HandleHasher<hParam>>;

class Expr {
public:

    enum class Op : uint32_t {
        // A parameter, by the hParam handle
        PARAM          =  0,
        // A parameter, by a pointer straight in to the param table (faster,
        // if we know that the param table won't move around)
        PARAM_PTR      =  1,

        // Operands
        CONSTANT       = 20,
        VARIABLE       = 21,
        // An indirectly-held constant: stores a `const double *` and
        // reads through it on every `Eval`. The pointer is stable for
        // the lifetime of whatever the caller pointed at. Treated as a
        // leaf with no children, derivative 0, and **not** a foldable
        // constant (its numeric value may change between Eval calls,
        // so `FoldConstants` must not fold past it). Used to embed
        // user-mutable values like `ConstraintBase::valA` in cached
        // Jacobian trees: the cache survives `Slvs_SetConstraintValue`
        // because the next `Eval` picks up the new value from the
        // same address.
        CONST_PTR      = 22,

        // Binary ops
        PLUS           = 100,
        MINUS          = 101,
        TIMES          = 102,
        DIV            = 103,
        // Unary ops
        NEGATE         = 104,
        SQRT           = 105,
        SQUARE         = 106,
        SIN            = 107,
        COS            = 108,
        ASIN           = 109,
        ACOS           = 110,
    };

    Op      op;
    Expr    *a;
    union {
        double        v;             // CONSTANT
        hParam        parh;          // PARAM
        Param        *parp;          // PARAM_PTR
        const double *const_ptr;     // CONST_PTR
        Expr         *b;             // binary ops
    };

    Expr() = default;
    Expr(double val) : op(Op::CONSTANT) { v = val; }

    static Expr *From(hParam p);
    static Expr *From(double v);
    // Build an indirect constant — the resulting Expr reads through
    // `*ptr` on each Eval. See Op::CONST_PTR.
    static Expr *FromPtr(const double *ptr);

    Expr *AnyOp(Op op, Expr *b);
    inline Expr *Plus (Expr *b_) { return AnyOp(Op::PLUS,  b_); }
    inline Expr *Minus(Expr *b_) { return AnyOp(Op::MINUS, b_); }
    inline Expr *Times(Expr *b_) { return AnyOp(Op::TIMES, b_); }
    inline Expr *Div  (Expr *b_) { return AnyOp(Op::DIV,   b_); }

    inline Expr *Negate() { return AnyOp(Op::NEGATE, NULL); }
    inline Expr *Sqrt  () { return AnyOp(Op::SQRT,   NULL); }
    inline Expr *Square() { return AnyOp(Op::SQUARE, NULL); }
    inline Expr *Sin   () { return AnyOp(Op::SIN,    NULL); }
    inline Expr *Cos   () { return AnyOp(Op::COS,    NULL); }
    inline Expr *ASin  () { return AnyOp(Op::ASIN,   NULL); }
    inline Expr *ACos  () { return AnyOp(Op::ACOS,   NULL); }

    Expr *PartialWrt(hParam p) const;
    // `sk` is consulted only when the expression tree still contains
    // Op::PARAM nodes (handle-form). After `DeepCopyWithParamsAsPointers`
    // substitutes those to Op::PARAM_PTR (direct double*), the hot Newton
    // loop calls Eval() with sk=nullptr because no lookup is needed.
    double Eval(const Sketch *sk = nullptr) const;
    void ParamsUsedList(ParamSet *list) const;
    bool DependsOn(hParam p) const;
    static bool Tol(double a, double b);
    bool IsZeroConst() const;
    Expr *FoldConstants(bool allocCopy = true, size_t depth = std::numeric_limits<size_t>::max());
    void Substitute(const SubstitutionMap &subMap);

    static const hParam NO_PARAMS, MULTIPLE_PARAMS;
    hParam ReferencedParams(ParamList *pl) const;

    void ParamsToPointers();

    std::string Print() const;

    // number of child nodes: 0 (e.g. constant), 1 (sqrt), or 2 (+)
    int Children() const;
    // total number of nodes in the tree
    int Nodes() const;

    // Make a simple copy
    Expr *DeepCopy() const;
    // Make a copy, with the parameters (usually referenced by hParam)
    // resolved to pointers to the actual value. This speeds things up
    // considerably.
    Expr *DeepCopyWithParamsAsPointers(ParamList *firstTry,
                                       ParamList *thenTry,
                                       bool foldConstants = false) const;
    // Deep-copy this tree allocating every node from `heap` (instead
    // of the thread's transient AllocTemporary heap). Used by the
    // Jacobian cache (Phase 2) to promote Exprs out of the per-solve
    // temp arena into the per-Solver persistent arena so they survive
    // `FreeAllTemporary`. Op::PARAM_PTR / Op::CONST_PTR leaves keep
    // their pointers — the caller is responsible for ensuring the
    // pointed-to Param / external double outlives the copy.
    Expr *DeepCopyIntoHeap(mi_heap_t *heap) const;

    static Expr *Parse(const std::string &input, std::string *error);
    static Expr *From(const std::string &input, bool popUpError);
};

class ExprVector {
public:
    Expr *x, *y, *z;

    static ExprVector From(Expr *x, Expr *y, Expr *z);
    static ExprVector From(Vector vn);
    static ExprVector From(hParam x, hParam y, hParam z);
    static ExprVector From(double x, double y, double z);

    ExprVector Plus(ExprVector b) const;
    ExprVector Minus(ExprVector b) const;
    Expr *Dot(ExprVector b) const;
    ExprVector Cross(ExprVector b) const;
    ExprVector ScaledBy(Expr *s) const;
    ExprVector WithMagnitude(Expr *s) const;
    Expr *Magnitude() const;

    Vector Eval(const Sketch *sk = nullptr) const;
};

class ExprQuaternion {
public:
    Expr *w, *vx, *vy, *vz;

    static ExprQuaternion From(Expr *w, Expr *vx, Expr *vy, Expr *vz);
    static ExprQuaternion From(Quaternion qn);
    static ExprQuaternion From(hParam w, hParam vx, hParam vy, hParam vz);

    ExprVector RotationU() const;
    ExprVector RotationV() const;
    ExprVector RotationN() const;

    ExprVector Rotate(ExprVector p) const;
    ExprQuaternion Times(ExprQuaternion b) const;

    Expr *Magnitude() const;
};

} // namespace SolveSpace

#endif
