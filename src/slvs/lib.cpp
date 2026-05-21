//-----------------------------------------------------------------------------
// A library wrapper around SolveSpace, to permit someone to use its constraint
// solver without coupling their program too much to SolveSpace's internals.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------
#include <algorithm>
#include "solvespace.h"
#include <slvs.h>
#include <string>

namespace SolveSpace {

// `Sketch SK`, `System SYS`, and `ParamSet dragged` are no longer
// process-globals. They moved into the per-instance `Solver` (see
// solver.h / solver.cpp). Existing `SK.foo`, `SYS.foo` references
// inside the internal C++ code still work via the macros in solver.h,
// which read `CurrentSolver` (a thread-local pointer).
//
// Public API contract: every `Slvs_*` data-mutating function takes a
// `Slvs_Solver *` as its first argument. On entry, the function sets
// `CurrentSolver` to that handle (saving + restoring the previous via
// the `WithCurrentSolver` RAII helper below) so that all SK / SYS uses
// inside route to the caller's solver. The thread-local is an
// implementation detail — it lets the existing ~290 SK/SYS references
// across constrainteq.cpp / entity.cpp / system.cpp keep working
// unchanged while the public API is fully explicit.

void Platform::FatalError(const std::string &message) {
    fprintf(stderr, "%s", message.c_str());
    abort();
}

void Group::GenerateEquations(IdList<Equation,hEquation> *) {
    // Nothing to do for now.
}

} // namespace SolveSpace

using namespace SolveSpace;

namespace {
// RAII: every public Slvs_* entry point selects its Solver as the
// thread's current via the constructor, restoring the previous on
// scope exit. Nested Slvs_* calls (e.g. `Slvs_Coincident` →
// `Slvs_AddConstraint` → `Slvs_AddParam`) compose correctly: each
// nested scope writes the same pointer it inherits, then restores it.
struct WithCurrentSolver {
    SolveSpace::Solver *prev;
    explicit WithCurrentSolver(Slvs_Solver *handle)
        : prev(SolveSpace::CurrentSolver) {
        SolveSpace::CurrentSolver =
            reinterpret_cast<SolveSpace::Solver *>(handle);
    }
    ~WithCurrentSolver() { SolveSpace::CurrentSolver = prev; }
    WithCurrentSolver(const WithCurrentSolver &)            = delete;
    WithCurrentSolver &operator=(const WithCurrentSolver &) = delete;
};
}  // namespace

extern "C" {

static ConstraintBase::Type Slvs_CTypeToConstraintBaseType(int type) {
    switch(type) {
case SLVS_C_POINTS_COINCIDENT:   return ConstraintBase::Type::POINTS_COINCIDENT;
case SLVS_C_PT_PT_DISTANCE:      return ConstraintBase::Type::PT_PT_DISTANCE;
case SLVS_C_PT_PLANE_DISTANCE:   return ConstraintBase::Type::PT_PLANE_DISTANCE;
case SLVS_C_PT_LINE_DISTANCE:    return ConstraintBase::Type::PT_LINE_DISTANCE;
case SLVS_C_PT_FACE_DISTANCE:    return ConstraintBase::Type::PT_FACE_DISTANCE;
case SLVS_C_PT_IN_PLANE:         return ConstraintBase::Type::PT_IN_PLANE;
case SLVS_C_PT_ON_LINE:          return ConstraintBase::Type::PT_ON_LINE;
case SLVS_C_PT_ON_FACE:          return ConstraintBase::Type::PT_ON_FACE;
case SLVS_C_EQUAL_LENGTH_LINES:  return ConstraintBase::Type::EQUAL_LENGTH_LINES;
case SLVS_C_LENGTH_RATIO:        return ConstraintBase::Type::LENGTH_RATIO;
case SLVS_C_ARC_ARC_LEN_RATIO:   return ConstraintBase::Type::ARC_ARC_LEN_RATIO;
case SLVS_C_ARC_LINE_LEN_RATIO:  return ConstraintBase::Type::ARC_LINE_LEN_RATIO;
case SLVS_C_EQ_LEN_PT_LINE_D:    return ConstraintBase::Type::EQ_LEN_PT_LINE_D;
case SLVS_C_EQ_PT_LN_DISTANCES:  return ConstraintBase::Type::EQ_PT_LN_DISTANCES;
case SLVS_C_EQUAL_ANGLE:         return ConstraintBase::Type::EQUAL_ANGLE;
case SLVS_C_EQUAL_LINE_ARC_LEN:  return ConstraintBase::Type::EQUAL_LINE_ARC_LEN;
case SLVS_C_LENGTH_DIFFERENCE:   return ConstraintBase::Type::LENGTH_DIFFERENCE;
case SLVS_C_ARC_ARC_DIFFERENCE:  return ConstraintBase::Type::ARC_ARC_DIFFERENCE;
case SLVS_C_ARC_LINE_DIFFERENCE: return ConstraintBase::Type::ARC_LINE_DIFFERENCE;
case SLVS_C_SYMMETRIC:           return ConstraintBase::Type::SYMMETRIC;
case SLVS_C_SYMMETRIC_HORIZ:     return ConstraintBase::Type::SYMMETRIC_HORIZ;
case SLVS_C_SYMMETRIC_VERT:      return ConstraintBase::Type::SYMMETRIC_VERT;
case SLVS_C_SYMMETRIC_LINE:      return ConstraintBase::Type::SYMMETRIC_LINE;
case SLVS_C_AT_MIDPOINT:         return ConstraintBase::Type::AT_MIDPOINT;
case SLVS_C_HORIZONTAL:          return ConstraintBase::Type::HORIZONTAL;
case SLVS_C_VERTICAL:            return ConstraintBase::Type::VERTICAL;
case SLVS_C_DIAMETER:            return ConstraintBase::Type::DIAMETER;
case SLVS_C_PT_ON_CIRCLE:        return ConstraintBase::Type::PT_ON_CIRCLE;
case SLVS_C_SAME_ORIENTATION:    return ConstraintBase::Type::SAME_ORIENTATION;
case SLVS_C_ANGLE:               return ConstraintBase::Type::ANGLE;
case SLVS_C_PARALLEL:            return ConstraintBase::Type::PARALLEL;
case SLVS_C_PERPENDICULAR:       return ConstraintBase::Type::PERPENDICULAR;
case SLVS_C_ARC_LINE_TANGENT:    return ConstraintBase::Type::ARC_LINE_TANGENT;
case SLVS_C_CUBIC_LINE_TANGENT:  return ConstraintBase::Type::CUBIC_LINE_TANGENT;
case SLVS_C_EQUAL_RADIUS:        return ConstraintBase::Type::EQUAL_RADIUS;
case SLVS_C_PROJ_PT_DISTANCE:    return ConstraintBase::Type::PROJ_PT_DISTANCE;
case SLVS_C_WHERE_DRAGGED:       return ConstraintBase::Type::WHERE_DRAGGED;
case SLVS_C_CURVE_CURVE_TANGENT: return ConstraintBase::Type::CURVE_CURVE_TANGENT;
default: Platform::FatalError("bad constraint type " + std::to_string(type));
    }
}

static EntityBase::Type Slvs_CTypeToEntityBaseType(int type) {
    switch(type) {
case SLVS_E_POINT_IN_3D:        return EntityBase::Type::POINT_IN_3D;
case SLVS_E_POINT_IN_2D:        return EntityBase::Type::POINT_IN_2D;
case SLVS_E_NORMAL_IN_3D:       return EntityBase::Type::NORMAL_IN_3D;
case SLVS_E_NORMAL_IN_2D:       return EntityBase::Type::NORMAL_IN_2D;
case SLVS_E_DISTANCE:           return EntityBase::Type::DISTANCE;
case SLVS_E_WORKPLANE:          return EntityBase::Type::WORKPLANE;
case SLVS_E_LINE_SEGMENT:       return EntityBase::Type::LINE_SEGMENT;
case SLVS_E_CUBIC:              return EntityBase::Type::CUBIC;
case SLVS_E_CIRCLE:             return EntityBase::Type::CIRCLE;
case SLVS_E_ARC_OF_CIRCLE:      return EntityBase::Type::ARC_OF_CIRCLE;
default: Platform::FatalError("bad entity type " + std::to_string(type));
    }
}

static bool Slvs_CanInitiallySatisfy(const ConstraintBase &c) {
    switch(c.type) {
    case ConstraintBase::Type::CUBIC_LINE_TANGENT:
    case ConstraintBase::Type::PARALLEL:
        // Can't initially satisfy if not projected onto a workplane
        return c.workplane != EntityBase::FREE_IN_3D;

    case ConstraintBase::Type::PT_PT_DISTANCE:
    case ConstraintBase::Type::PROJ_PT_DISTANCE:
    case ConstraintBase::Type::PT_LINE_DISTANCE:
    case ConstraintBase::Type::PT_PLANE_DISTANCE:
    case ConstraintBase::Type::PT_FACE_DISTANCE:
    case ConstraintBase::Type::EQUAL_LENGTH_LINES:
    case ConstraintBase::Type::EQ_LEN_PT_LINE_D:
    case ConstraintBase::Type::EQ_PT_LN_DISTANCES:
    case ConstraintBase::Type::LENGTH_RATIO:
    case ConstraintBase::Type::ARC_ARC_LEN_RATIO:
    case ConstraintBase::Type::ARC_LINE_LEN_RATIO:
    case ConstraintBase::Type::LENGTH_DIFFERENCE:
    case ConstraintBase::Type::ARC_ARC_DIFFERENCE:
    case ConstraintBase::Type::ARC_LINE_DIFFERENCE:
    case ConstraintBase::Type::DIAMETER:
    case ConstraintBase::Type::EQUAL_RADIUS:
    case ConstraintBase::Type::EQUAL_LINE_ARC_LEN:
    case ConstraintBase::Type::PT_IN_PLANE:
    case ConstraintBase::Type::PT_ON_FACE:
    case ConstraintBase::Type::PT_ON_CIRCLE:
    case ConstraintBase::Type::HORIZONTAL:
    case ConstraintBase::Type::VERTICAL:
    case ConstraintBase::Type::PERPENDICULAR:
    case ConstraintBase::Type::ANGLE:
    case ConstraintBase::Type::EQUAL_ANGLE:
    case ConstraintBase::Type::ARC_LINE_TANGENT:
    case ConstraintBase::Type::CURVE_CURVE_TANGENT:
        return true;

    case ConstraintBase::Type::AT_MIDPOINT:
        // Can initially satisfy if between a line segment and a workplane
        return c.ptA == EntityBase::NO_ENTITY;

    case ConstraintBase::Type::POINTS_COINCIDENT:
    case ConstraintBase::Type::PT_ON_LINE:
    case ConstraintBase::Type::SYMMETRIC:
    case ConstraintBase::Type::SYMMETRIC_HORIZ:
    case ConstraintBase::Type::SYMMETRIC_VERT:
    case ConstraintBase::Type::SYMMETRIC_LINE:
    case ConstraintBase::Type::SAME_ORIENTATION:
    case ConstraintBase::Type::WHERE_DRAGGED:
    case ConstraintBase::Type::COMMENT:
        // Can't initially satisfy these constraints
        return false;
    }
    ssassert(false, "Unexpected constraint type");
}

bool Slvs_IsFreeIn3D(Slvs_Entity e) {
    return e.h == SLVS_FREE_IN_3D;
}

bool Slvs_Is3D(Slvs_Entity e) {
    return e.wrkpl == SLVS_FREE_IN_3D;
}

bool Slvs_IsNone(Slvs_Entity e) {
    return e.h == 0;
}

bool Slvs_IsPoint2D(Slvs_Entity e) {
    return e.type == SLVS_E_POINT_IN_2D;
}

bool Slvs_IsPoint3D(Slvs_Entity e) {
    return e.type == SLVS_E_POINT_IN_3D;
}

bool Slvs_IsNormal2D(Slvs_Entity e) {
    return e.type == SLVS_E_NORMAL_IN_2D;
}

bool Slvs_IsNormal3D(Slvs_Entity e) {
    return e.type == SLVS_E_NORMAL_IN_3D;
}

bool Slvs_IsLine(Slvs_Entity e) {
    return e.type == SLVS_E_LINE_SEGMENT;
}

bool Slvs_IsLine2D(Slvs_Entity e) {
    return e.type == SLVS_E_LINE_SEGMENT && !Slvs_Is3D(e);
}

bool Slvs_IsLine3D(Slvs_Entity e) {
    return e.type == SLVS_E_LINE_SEGMENT && Slvs_Is3D(e);
}

bool Slvs_IsCubic(Slvs_Entity e) {
    return e.type == SLVS_E_CUBIC;
}

bool Slvs_IsArc(Slvs_Entity e) {
    return e.type == SLVS_E_ARC_OF_CIRCLE;
}

bool Slvs_IsWorkplane(Slvs_Entity e) {
    return e.type == SLVS_E_WORKPLANE;
}

bool Slvs_IsDistance(Slvs_Entity e) {
    return e.type == SLVS_E_DISTANCE;
}

bool Slvs_IsPoint(Slvs_Entity e) {
    switch(e.type) {
        case SLVS_E_POINT_IN_3D:
        case SLVS_E_POINT_IN_2D:
            return true;
        default:
            return false;
    }
}

bool Slvs_IsCircle(Slvs_Entity e) {
    return e.type == SLVS_E_CIRCLE || e.type == SLVS_E_ARC_OF_CIRCLE;
}

// File-local helper. Always called from inside a public Slvs_* entry
// point (which has already pinned the right `CurrentSolver`), so it
// has no Slvs_Solver* parameter of its own — it reads SK via the macro.
static Slvs_hParam Slvs_AddParam(double val) {
    Param pa = {};
    pa.val   = val;
    SK.param.AddAndAssignId(&pa);
    return pa.h.v;
}

// entities
Slvs_Entity Slvs_AddPoint2D(Slvs_Solver *solver, uint32_t grouph, double u, double v, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    Slvs_hParam uph      = Slvs_AddParam(u);
    Slvs_hParam vph      = Slvs_AddParam(v);
    EntityBase e  = {};
    e.type        = EntityBase::Type::POINT_IN_2D;
    e.group.v     = grouph;
    e.workplane.v = workplane.h;
    e.param[0].v  = uph;
    e.param[1].v  = vph;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_POINT_IN_2D;
    ce.group = grouph;
    ce.wrkpl = workplane.h;
    ce.param[0] = uph;
    ce.param[1] = vph;
    return ce;
}

Slvs_Entity Slvs_AddPoint3D(Slvs_Solver *solver, uint32_t grouph, double x, double y, double z) {
    WithCurrentSolver _ws(solver);
    Slvs_hParam xph      = Slvs_AddParam(x);
    Slvs_hParam yph      = Slvs_AddParam(y);
    Slvs_hParam zph      = Slvs_AddParam(z);
    EntityBase e  = {};
    e.type        = EntityBase::Type::POINT_IN_3D;
    e.group.v     = grouph;
    e.workplane.v = EntityBase::FREE_IN_3D.v;
    e.param[0].v  = xph;
    e.param[1].v  = yph;
    e.param[2].v  = zph;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_POINT_IN_3D;
    ce.group = grouph;
    ce.wrkpl = SLVS_FREE_IN_3D;
    ce.param[0] = xph;
    ce.param[1] = yph;
    ce.param[2] = zph;
    return ce;
}

Slvs_Entity Slvs_AddNormal2D(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(!Slvs_IsWorkplane(workplane)) {
        Platform::FatalError("workplane argument is not a workplane");
    }
    EntityBase e  = {};
    e.type        = EntityBase::Type::NORMAL_IN_2D;
    e.group.v     = grouph;
    e.workplane.v = workplane.h;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_NORMAL_IN_2D;
    ce.group = grouph;
    ce.wrkpl = workplane.h;
    return ce;
}

Slvs_Entity Slvs_AddNormal3D(Slvs_Solver *solver, uint32_t grouph, double qw, double qx, double qy, double qz) {
    WithCurrentSolver _ws(solver);
    Slvs_hParam wph      = Slvs_AddParam(qw);
    Slvs_hParam xph      = Slvs_AddParam(qx);
    Slvs_hParam yph      = Slvs_AddParam(qy);
    Slvs_hParam zph      = Slvs_AddParam(qz);
    EntityBase e  = {};
    e.type        = EntityBase::Type::NORMAL_IN_3D;
    e.group.v     = grouph;
    e.workplane.v = EntityBase::FREE_IN_3D.v;
    e.param[0].v  = wph;
    e.param[1].v  = xph;
    e.param[2].v  = yph;
    e.param[3].v  = zph;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_NORMAL_IN_3D;
    ce.group = grouph;
    ce.wrkpl = SLVS_FREE_IN_3D;
    ce.param[0] = wph;
    ce.param[1] = xph;
    ce.param[2] = yph;
    ce.param[3] = zph;
    return ce;
}

Slvs_Entity Slvs_AddDistance(Slvs_Solver *solver, uint32_t grouph, double value, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(!Slvs_IsWorkplane(workplane)) {
        Platform::FatalError("workplane argument is not a workplane");
    }
    Slvs_hParam valueph  = Slvs_AddParam(value);
    EntityBase e  = {};
    e.type        = EntityBase::Type::DISTANCE;
    e.group.v     = grouph;
    e.workplane.v = workplane.h;
    e.param[0].v  = valueph;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_DISTANCE;
    ce.group = grouph;
    ce.wrkpl = workplane.h;
    ce.param[0] = valueph;
    return ce;
}

Slvs_Entity Slvs_AddLine2D(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(!Slvs_IsWorkplane(workplane)) {
        Platform::FatalError("workplane argument is not a workplane");
    } else if(!Slvs_IsPoint2D(ptA)) {
        Platform::FatalError("ptA argument is not a 2d point");
    } else if(!Slvs_IsPoint2D(ptB)) {
        Platform::FatalError("ptB argument is not a 2d point");
    }
    EntityBase e  = {};
    e.type        = EntityBase::Type::LINE_SEGMENT;
    e.group.v     = grouph;
    e.workplane.v = workplane.h;
    e.point[0].v  = ptA.h;
    e.point[1].v  = ptB.h;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_LINE_SEGMENT;
    ce.group = grouph;
    ce.wrkpl = workplane.h;
    ce.point[0] = ptA.h;
    ce.point[1] = ptB.h;
    return ce;
}

Slvs_Entity Slvs_AddLine3D(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity ptB) {
    WithCurrentSolver _ws(solver);
    if(!Slvs_IsPoint3D(ptA)) {
        Platform::FatalError("ptA argument is not a 3d point");
    } else if(!Slvs_IsPoint3D(ptB)) {
        Platform::FatalError("ptB argument is not a 3d point");
    }
    EntityBase e  = {};
    e.type        = EntityBase::Type::LINE_SEGMENT;
    e.group.v     = grouph;
    e.workplane.v = EntityBase::FREE_IN_3D.v;
    e.point[0].v  = ptA.h;
    e.point[1].v  = ptB.h;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_LINE_SEGMENT;
    ce.group = grouph;
    ce.wrkpl = SLVS_FREE_IN_3D;
    ce.point[0] = ptA.h;
    ce.point[1] = ptB.h;
    return ce;
}

Slvs_Entity Slvs_AddCubic(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity ptC, Slvs_Entity ptD, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(!Slvs_IsWorkplane(workplane)) {
        Platform::FatalError("workplane argument is not a workplane");
    } else if(!Slvs_IsPoint2D(ptA)) {
        Platform::FatalError("ptA argument is not a 2d point");
    } else if(!Slvs_IsPoint2D(ptB)) {
        Platform::FatalError("ptB argument is not a 2d point");
    } else if(!Slvs_IsPoint2D(ptC)) {
        Platform::FatalError("ptC argument is not a 2d point");
    } else if(!Slvs_IsPoint2D(ptD)) {
        Platform::FatalError("ptD argument is not a 2d point");
    }
    EntityBase e  = {};
    e.type        = EntityBase::Type::CUBIC;
    e.group.v     = grouph;
    e.workplane.v = workplane.h;
    e.point[0].v  = ptA.h;
    e.point[1].v  = ptB.h;
    e.point[2].v  = ptC.h;
    e.point[3].v  = ptD.h;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_CUBIC;
    ce.group = grouph;
    ce.wrkpl = workplane.h;
    ce.point[0] = ptA.h;
    ce.point[1] = ptB.h;
    ce.point[2] = ptC.h;
    ce.point[3] = ptD.h;
    return ce;
}


Slvs_Entity Slvs_AddArc(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity normal, Slvs_Entity center, Slvs_Entity start, Slvs_Entity end,
                            Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(!Slvs_IsWorkplane(workplane)) {
        Platform::FatalError("workplane argument is not a workplane");
    } else if(!Slvs_IsNormal3D(normal)) {
        Platform::FatalError("normal argument is not a 3d normal");
    } else if(!Slvs_IsPoint2D(center)) {
        Platform::FatalError("center argument is not a 2d point");
    } else if(!Slvs_IsPoint2D(start)) {
        Platform::FatalError("start argument is not a 2d point");
    } else if(!Slvs_IsPoint2D(end)) {
        Platform::FatalError("end argument is not a 2d point");
    }
    EntityBase e  = {};
    e.type        = EntityBase::Type::ARC_OF_CIRCLE;
    e.group.v     = grouph;
    e.workplane.v = workplane.h;
    e.normal.v    = normal.h;
    e.point[0].v  = center.h;
    e.point[1].v  = start.h;
    e.point[2].v  = end.h;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_ARC_OF_CIRCLE;
    ce.group = grouph;
    ce.wrkpl = workplane.h;
    ce.normal = normal.h;
    ce.point[0] = center.h;
    ce.point[1] = start.h;
    ce.point[2] = end.h;
    return ce;
}

Slvs_Entity Slvs_AddCircle(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity normal, Slvs_Entity center, Slvs_Entity radius,
                            Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(!Slvs_IsWorkplane(workplane)) {
        Platform::FatalError("workplane argument is not a workplane");
    } else if(!Slvs_IsNormal3D(normal)) {
        Platform::FatalError("normal argument is not a 3d normal");
    } else if(!Slvs_IsPoint2D(center)) {
        Platform::FatalError("center argument is not a 2d point");
    } else if(!Slvs_IsDistance(radius)) {
        Platform::FatalError("radius argument is not a distance");
    }
    EntityBase e  = {};
    e.type        = EntityBase::Type::CIRCLE;
    e.group.v     = grouph;
    e.workplane.v = workplane.h;
    e.normal.v    = normal.h;
    e.point[0].v  = center.h;
    e.distance.v  = radius.h;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_CIRCLE;
    ce.group = grouph;
    ce.wrkpl = workplane.h;
    ce.normal = normal.h;
    ce.point[0] = center.h;
    ce.distance = radius.h;
    return ce;
}

Slvs_Entity Slvs_AddWorkplane(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity origin, Slvs_Entity nm) {
    WithCurrentSolver _ws(solver);
    EntityBase e  = {};
    e.type        = EntityBase::Type::WORKPLANE;
    e.group.v     = grouph;
    e.workplane.v = SLVS_FREE_IN_3D;
    e.point[0].v  = origin.h;
    e.normal.v    = nm.h;
    SK.entity.AddAndAssignId(&e);

    Slvs_Entity ce = Slvs_Entity {};
    ce.h = e.h.v;
    ce.type = SLVS_E_WORKPLANE;
    ce.group = grouph;
    ce.wrkpl = SLVS_FREE_IN_3D;
    ce.point[0] = origin.h;
    ce.normal = nm.h;
    return ce;
}

Slvs_Entity Slvs_AddBase2D(Slvs_Solver *solver, uint32_t grouph) {
    WithCurrentSolver _ws(solver);
    Vector u      = Vector::From(1, 0, 0);
    Vector v      = Vector::From(0, 1, 0);
    Quaternion q  = Quaternion::From(u, v);
    Slvs_Entity nm = Slvs_AddNormal3D(solver, grouph, q.w, q.vx, q.vy, q.vz);
    return Slvs_AddWorkplane(solver, grouph, Slvs_AddPoint3D(solver, grouph, 0, 0, 0), nm);
}

// constraints

Slvs_Constraint Slvs_AddConstraint(Slvs_Solver *solver, uint32_t grouph,
    int type, Slvs_Entity workplane, double val, Slvs_Entity ptA,
    Slvs_Entity ptB = SLVS_E_NONE, Slvs_Entity entityA = SLVS_E_NONE,
    Slvs_Entity entityB = SLVS_E_NONE, Slvs_Entity entityC = SLVS_E_NONE,
    Slvs_Entity entityD = SLVS_E_NONE, int other = 0, int other2 = 0) {
    WithCurrentSolver _ws(solver);
    ConstraintBase c = {};
    c.type           = Slvs_CTypeToConstraintBaseType(type);
    c.group.v        = grouph;
    c.workplane.v    = workplane.h;
    c.valA           = val;
    c.ptA.v          = ptA.h;
    c.ptB.v          = ptB.h;
    c.entityA.v      = entityA.h;
    c.entityB.v      = entityB.h;
    c.entityC.v      = entityC.h;
    c.entityD.v      = entityD.h;
    c.other          = other ? true : false;
    c.other2         = other2 ? true : false;
    SK.constraint.AddAndAssignId(&c);

    Slvs_Constraint cc = Slvs_Constraint {};
    cc.h = c.h.v;
    cc.type = type;
    cc.group = grouph;
    cc.wrkpl = workplane.h;
    cc.valA = val;
    cc.ptA = ptA.h;
    cc.ptB = ptB.h;
    cc.entityA = entityA.h;
    cc.entityB = entityB.h;
    cc.entityC = entityC.h;
    cc.entityD = entityD.h;
    cc.other = other ? true : false;
    cc.other2 = other2 ? true : false;
    return cc;
}

Slvs_Constraint Slvs_Coincident(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint(entityA) && Slvs_IsPoint(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_POINTS_COINCIDENT, workplane, 0., entityA, entityB);
    } else if(Slvs_IsPoint(entityA) && Slvs_IsWorkplane(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PT_IN_PLANE, SLVS_E_FREE_IN_3D, 0., entityA, SLVS_E_NONE, entityB);
    } else if(Slvs_IsPoint(entityA) && Slvs_IsLine(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PT_ON_LINE, workplane, 0., entityA, SLVS_E_NONE, entityB);
    } else if(Slvs_IsPoint(entityA) && (Slvs_IsCircle(entityB) || Slvs_IsArc(entityB))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PT_ON_CIRCLE, workplane, 0., entityA, SLVS_E_NONE, entityB);
    }
    Platform::FatalError("Invalid arguments for coincident constraint");
}

Slvs_Constraint Slvs_Distance(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint(entityA) && Slvs_IsPoint(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PT_PT_DISTANCE, workplane, value, entityA, entityB);
    } else if(Slvs_IsPoint(entityA) && Slvs_IsWorkplane(entityB) && Slvs_Is3D(workplane)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PT_PLANE_DISTANCE, entityB, value, entityA, SLVS_E_NONE, entityB);
    } else if(Slvs_IsPoint(entityA) && Slvs_IsLine(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PT_LINE_DISTANCE, workplane, value, entityA, SLVS_E_NONE, entityB);
    }
    Platform::FatalError("Invalid arguments for distance constraint");
}

Slvs_Constraint Slvs_Equal(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsLine(entityA) && Slvs_IsLine(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_EQUAL_LENGTH_LINES, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB);
    } else if(Slvs_IsLine(entityA) && (Slvs_IsArc(entityB) || Slvs_IsCircle(entityB))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_EQUAL_LINE_ARC_LEN, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB);
    } else if((Slvs_IsArc(entityA) || Slvs_IsCircle(entityA)) && (Slvs_IsArc(entityB) || Slvs_IsCircle(entityB))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_EQUAL_RADIUS, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB);
    }
    Platform::FatalError("Invalid arguments for equal constraint");
}

Slvs_Constraint Slvs_EqualAngle(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity entityC, Slvs_Entity entityD, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsLine2D(entityA) && Slvs_IsLine2D(entityB) && Slvs_IsLine2D(entityC) && Slvs_IsLine2D(entityD) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_EQUAL_ANGLE, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB, entityC, entityD);
    }
    Platform::FatalError("Invalid arguments for equal angle constraint");
}

Slvs_Constraint Slvs_EqualPointToLine(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity entityC, Slvs_Entity entityD, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint2D(entityA) && Slvs_IsLine2D(entityB) && Slvs_IsPoint2D(entityC) && Slvs_IsLine2D(entityD) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_EQ_PT_LN_DISTANCES, workplane, 0., entityA, entityB, entityC, entityD);
    }
    Platform::FatalError("Invalid arguments for equal point to line constraint");
}

Slvs_Constraint Slvs_Ratio(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsLine2D(entityA) && Slvs_IsLine2D(entityB) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_LENGTH_RATIO, workplane, value, SLVS_E_NONE, SLVS_E_NONE, entityA, entityB);
    }
    Platform::FatalError("Invalid arguments for ratio constraint");
}

Slvs_Constraint Slvs_Symmetric(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity entityC = SLVS_E_NONE, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint3D(entityA) && Slvs_IsPoint3D(entityB) && Slvs_IsWorkplane(entityC) && Slvs_IsFreeIn3D(workplane)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_SYMMETRIC, workplane, 0., entityA, entityB, entityC);
    } else if(Slvs_IsPoint2D(entityA) && Slvs_IsPoint2D(entityB) && Slvs_IsWorkplane(entityC) && Slvs_IsFreeIn3D(workplane)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_SYMMETRIC, entityC, 0., entityA, entityB, entityC);
    } else if(Slvs_IsPoint2D(entityA) && Slvs_IsPoint2D(entityB) && Slvs_IsLine(entityC)) {
        if(Slvs_IsFreeIn3D(workplane)) {
            Platform::FatalError("3d workplane given for a 2d constraint");
        }
        return Slvs_AddConstraint(solver, grouph, SLVS_C_SYMMETRIC_LINE, workplane, 0., entityA, entityB, entityC);
    }
    Platform::FatalError("Invalid arguments for symmetric constraint");
}

Slvs_Constraint Slvs_SymmetricH(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsFreeIn3D(workplane)) {
        Platform::FatalError("3d workplane given for a 2d constraint");
    } else if(Slvs_IsPoint2D(ptA) && Slvs_IsPoint2D(ptB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_SYMMETRIC_HORIZ, workplane, 0., ptA, ptB);
    }
    Platform::FatalError("Invalid arguments for symmetric horizontal constraint");
}

Slvs_Constraint Slvs_SymmetricV(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsFreeIn3D(workplane)) {
        Platform::FatalError("3d workplane given for a 2d constraint");
    } else if(Slvs_IsPoint2D(ptA) && Slvs_IsPoint2D(ptB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_SYMMETRIC_VERT, workplane, 0., ptA, ptB);
    }
    Platform::FatalError("Invalid arguments for symmetric vertical constraint");
}

Slvs_Constraint Slvs_Midpoint(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint(ptA) && Slvs_IsLine(ptB) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_AT_MIDPOINT, workplane, 0., ptA, SLVS_E_NONE, ptB);
    }
    Platform::FatalError("Invalid arguments for midpoint constraint");
}

Slvs_Constraint Slvs_Horizontal(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity workplane, Slvs_Entity entityB = SLVS_E_NONE) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsFreeIn3D(workplane)) {
        Platform::FatalError("Horizontal constraint is not supported in 3D");
    } else if(Slvs_IsLine2D(entityA)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_HORIZONTAL, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA);
    } else if(Slvs_IsPoint2D(entityA) && Slvs_IsPoint2D(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_HORIZONTAL, workplane, 0., entityA, entityB);
    }
    Platform::FatalError("Invalid arguments for horizontal constraint");
}

Slvs_Constraint Slvs_Vertical(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity workplane, Slvs_Entity entityB = SLVS_E_NONE) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsFreeIn3D(workplane)) {
        Platform::FatalError("Vertical constraint is not supported in 3D");
    } else if(Slvs_IsLine2D(entityA)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_VERTICAL, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA);
    } else if(Slvs_IsPoint2D(entityA) && Slvs_IsPoint2D(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_VERTICAL, workplane, 0., entityA, entityB);
    }
    Platform::FatalError("Invalid arguments for horizontal constraint");
}

Slvs_Constraint Slvs_Diameter(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, double value) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsArc(entityA) || Slvs_IsCircle(entityA)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_DIAMETER, SLVS_E_FREE_IN_3D, value, SLVS_E_NONE, SLVS_E_NONE, entityA);
    }
    Platform::FatalError("Invalid arguments for diameter constraint");
}

Slvs_Constraint Slvs_SameOrientation(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsNormal3D(entityA) && Slvs_IsNormal3D(entityB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_SAME_ORIENTATION, SLVS_E_FREE_IN_3D, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB);
    }
    Platform::FatalError("Invalid arguments for same orientation constraint");
}

Slvs_Constraint Slvs_Angle(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane = SLVS_E_FREE_IN_3D, int inverse = 0) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsLine2D(entityA) && Slvs_IsLine2D(entityB) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_ANGLE, workplane, value, SLVS_E_NONE, SLVS_E_NONE, entityA, entityB, SLVS_E_NONE, SLVS_E_NONE, inverse);
    }
    Platform::FatalError("Invalid arguments for angle constraint");
}

Slvs_Constraint Slvs_Perpendicular(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane = SLVS_E_FREE_IN_3D, int inverse = 0) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsLine2D(entityA) && Slvs_IsLine2D(entityB) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PERPENDICULAR, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB, SLVS_E_NONE, SLVS_E_NONE, inverse);
    }
    Platform::FatalError("Invalid arguments for perpendicular constraint");
}

Slvs_Constraint Slvs_Parallel(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsLine2D(entityA) && Slvs_IsLine2D(entityB) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PARALLEL, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB);
    }
    Platform::FatalError("Invalid arguments for parallel constraint");
}

Slvs_Constraint Slvs_Tangent(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsArc(entityA) && Slvs_IsLine2D(entityB)) {
        if(Slvs_IsFreeIn3D(workplane)) {
            Platform::FatalError("3d workplane given for a 2d constraint");
        }
        Vector a1 = SK.entity.FindById(hEntity { entityA.point[1] })->PointGetNum(),
               a2 = SK.entity.FindById(hEntity { entityA.point[2] })->PointGetNum();
        Vector l0 = SK.entity.FindById(hEntity { entityB.point[0] })->PointGetNum(),
               l1 = SK.entity.FindById(hEntity { entityB.point[1] })->PointGetNum();
        int other;
        if(l0.Equals(a1) || l1.Equals(a1)) {
            other = 0;
        } else if(l0.Equals(a2) || l1.Equals(a2)) {
            other = 1;
        } else {
            Platform::FatalError("The tangent arc and line segment must share an "
                                        "endpoint. Constrain them with Constrain -> "
                                        "On Point before constraining tangent.");
        }
        return Slvs_AddConstraint(solver, grouph, SLVS_C_ARC_LINE_TANGENT, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB, SLVS_E_NONE, SLVS_E_NONE, other);
    } else if(Slvs_IsCubic(entityA) && Slvs_IsLine2D(entityB) && Slvs_IsFreeIn3D(workplane)) {
        EntityBase* skEntityA = SK.entity.FindById(hEntity { entityA.h });
        Vector as = skEntityA->CubicGetStartNum(), af = skEntityA->CubicGetFinishNum();
        Vector l0 = SK.entity.FindById(hEntity { entityB.point[0] })->PointGetNum(),
               l1 = SK.entity.FindById(hEntity { entityB.point[1] })->PointGetNum();
        int other;
        if(l0.Equals(as) || l1.Equals(as)) {
            other = 0;
        } else if(l0.Equals(af) || l1.Equals(af)) {
            other = 1;
        } else {
            Platform::FatalError("The tangent cubic and line segment must share an "
                                        "endpoint. Constrain them with Constrain -> "
                                        "On Point before constraining tangent.");
        }
        return Slvs_AddConstraint(solver, grouph, SLVS_C_CUBIC_LINE_TANGENT, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB, SLVS_E_NONE, SLVS_E_NONE, other);
    } else if((Slvs_IsArc(entityA) || Slvs_IsCubic(entityA)) && (Slvs_IsArc(entityB) || Slvs_IsCubic(entityB))) {
        if(Slvs_IsFreeIn3D(workplane)) {
            Platform::FatalError("3d workplane given for a 2d constraint");
        }
        EntityBase* skEntityA = SK.entity.FindById(hEntity { entityA.h });
        EntityBase* skEntityB = SK.entity.FindById(hEntity { entityB.h });
        Vector as = skEntityA->EndpointStart(), af = skEntityA->EndpointFinish(),
               bs = skEntityB->EndpointStart(), bf = skEntityB->EndpointFinish();
        int other;
        int other2;
        if(as.Equals(bs)) {
            other  = 0;
            other2 = 0;
        } else if(as.Equals(bf)) {
            other  = 0;
            other2 = 1;
        } else if(af.Equals(bs)) {
            other  = 1;
            other2 = 0;
        } else if(af.Equals(bf)) {
            other  = 1;
            other2 = 1;
        } else {
            Platform::FatalError("The curves must share an endpoint. Constrain them "
                                        "with Constrain -> On Point before constraining "
                                        "tangent.");
        }
        return Slvs_AddConstraint(solver, grouph, SLVS_C_CURVE_CURVE_TANGENT, workplane, 0., SLVS_E_NONE, SLVS_E_NONE, entityA, entityB, SLVS_E_NONE, SLVS_E_NONE, other, other2);
    }
    Platform::FatalError("Invalid arguments for tangent constraint");
}

Slvs_Constraint Slvs_DistanceProj(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity ptB, double value) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint(ptA) && Slvs_IsPoint(ptB)) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_PROJ_PT_DISTANCE, SLVS_E_FREE_IN_3D, value, ptA, ptB);
    }
    Platform::FatalError("Invalid arguments for projected distance constraint");
}

Slvs_Constraint Slvs_LengthDiff(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsLine(entityA) && Slvs_IsLine(entityB) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_LENGTH_DIFFERENCE, workplane, value, SLVS_E_NONE, SLVS_E_NONE, entityA, entityB);
    }
    Platform::FatalError("Invalid arguments for length difference constraint");
}

Slvs_Constraint Slvs_Dragged(Slvs_Solver *solver, uint32_t grouph, Slvs_Entity ptA, Slvs_Entity workplane = SLVS_E_FREE_IN_3D) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint(ptA) && (Slvs_IsWorkplane(workplane) || Slvs_IsFreeIn3D(workplane))) {
        return Slvs_AddConstraint(solver, grouph, SLVS_C_WHERE_DRAGGED, workplane, 0., ptA);
    }
    Platform::FatalError("Invalid arguments for dragged constraint");
}

void Slvs_QuaternionU(double qw, double qx, double qy, double qz,
                         double *x, double *y, double *z)
{
    Quaternion q = Quaternion::From(qw, qx, qy, qz);
    Vector v = q.RotationU();
    *x = v.x;
    *y = v.y;
    *z = v.z;
}

void Slvs_QuaternionV(double qw, double qx, double qy, double qz,
                         double *x, double *y, double *z)
{
    Quaternion q = Quaternion::From(qw, qx, qy, qz);
    Vector v = q.RotationV();
    *x = v.x;
    *y = v.y;
    *z = v.z;
}

void Slvs_QuaternionN(double qw, double qx, double qy, double qz,
                         double *x, double *y, double *z)
{
    Quaternion q = Quaternion::From(qw, qx, qy, qz);
    Vector v = q.RotationN();
    *x = v.x;
    *y = v.y;
    *z = v.z;
}

void Slvs_MakeQuaternion(double ux, double uy, double uz,
                         double vx, double vy, double vz,
                         double *qw, double *qx, double *qy, double *qz)
{
    Vector u = Vector::From(ux, uy, uz),
           v = Vector::From(vx, vy, vz);
    Quaternion q = Quaternion::From(u, v);
    *qw = q.w;
    *qx = q.vx;
    *qy = q.vy;
    *qz = q.vz;
}

void Slvs_ClearSketch(Slvs_Solver *solver)
{
    WithCurrentSolver _ws(solver);
    // Former globals — now per-Solver fields. See solver.h.
    EnsureCurrentSolver().dragged->clear();
    SYS.Clear();
    SK.param.Clear();
    SK.entity.Clear();
    SK.constraint.Clear();
}

void Slvs_MarkDragged(Slvs_Solver *solver, Slvs_Entity ptA) {
    WithCurrentSolver _ws(solver);
    if(Slvs_IsPoint(ptA)) {
        const size_t params = Slvs_IsPoint3D(ptA) ? 3 : 2;
        ParamSet &drag = *EnsureCurrentSolver().dragged;
        for(size_t i = 0; i < params; ++i) {
            hParam p = hParam { ptA.param[i] };
            drag.insert(p);
        }
    } else {
        SolveSpace::Platform::FatalError("Invalid entity for marking dragged");
    }
}

Slvs_SolveResult Slvs_SolveSketch(Slvs_Solver *solver, uint32_t shg, Slvs_hConstraint **bad = nullptr)
{
    WithCurrentSolver _ws(solver);
    SYS.Clear();

    Group g = {};
    g.h.v = shg;

    // add params from entities on sketch
    for(EntityBase &ent : SK.entity) {
        EntityBase *e = &ent;
        // skip entities from other groups
        if (e->group.v != shg) {
            continue;
        }
        for (hParam &parh : e->param) {
            if (parh.v != 0) {
                // get params for this entity and add it to the system
                Param *p = SK.GetParam(parh);
                p->known = false;
                SYS.param.Add(p);
            }
        }
    }

    // add params from constraints
    for(ConstraintBase &con : SK.constraint) {
        ConstraintBase *c = &con;
        if(c->group.v != shg)
            continue;
        // If we're solving a sketch twice without calling `Slvs_ClearSketch()` in between,
        // we already have a constraint param in the sketch, and regeneration would simply
        // create another one and orphan the existing one. While this doesn't create any
        // correctness issues, it does waste memory, so identify this case and regenerate
        // only if we actually need to.
        if(c->valP.v) {
            // Reset `known=false` so the solver treats this constraint-internal
            // param as an unknown on re-solve. `System::Solve` sets `known=true`
            // on every solved param at the end of a successful solve, and
            // `Expr::DeepCopyWithParamsAsPointers` folds any `known` param into
            // a CONSTANT. Without this reset, on the second `Slvs_SolveSketch`
            // call PT_ON_LINE / PARALLEL / SAME_ORIENTATION constraint params
            // would be frozen at their previous values → INCONSISTENT.
            // Mirrors the entity-param reset above.
            Param *p = SK.GetParam(c->valP);
            p->known = false;
            SYS.param.Add(p);
            continue;
        }
        // If `valP` is 0, this is either a constraint which doesn't have a param, or one
        // which we haven't seen before, so try to regenerate.
        // This generates at most a single additional param
        c->Generate(&SK.param);
        if(c->valP.v) {
            Param *p = SK.GetParam(c->valP);
            p->known = false;
            SYS.param.Add(p);

            if(Slvs_CanInitiallySatisfy(*c)) {
                c->ModifyToSatisfy();
            }
        }
    }

    // mark dragged params — pull from the current Solver's set into SYS.
    for(hParam p : *EnsureCurrentSolver().dragged) {
        SYS.dragged.insert(p);
    }

    // for(hParam &par : SYS.dragged) {
    //     std::cout << "DraggedParam( h:" << par.v << " )\n";
    // }

    // for(Param &par : SYS.param) {
    //     std::cout << "SysParam( " << par.ToString() << " )\n";
    // }

    // for(EntityBase &ent : SK.entity) {
    //     std::cout << "SketchEntityBase( " << ent.ToString() << " )\n";
    // }

    // for(ConstraintBase &con : SK.constraint) {
    //     std::cout << "SketchConstraintBase( " << con.ToString() << " )\n";
    // }

    List<hConstraint> badList;
    bool andFindBad = bad != nullptr;

    int dof = 0;
    SolveResult status = SYS.Solve(&g, &dof, &badList, andFindBad, false, false);
    Slvs_SolveResult sr = {};
    sr.dof = dof;
    sr.nbad = badList.n;
    if(bad) {
        if(sr.nbad <= 0) {
            *bad = nullptr;
        } else {
            *bad = static_cast<Slvs_hConstraint *>(malloc(sizeof(Slvs_hConstraint) * sr.nbad));
            for(int i = 0; i < sr.nbad; ++i) {
                (*bad)[i] = badList[i].v;
            }
        }
    }
    sr.result = 0;
    switch(status) {
        case SolveResult::OKAY:                    sr.result = SLVS_RESULT_OKAY; break;
        case SolveResult::DIDNT_CONVERGE:          sr.result = SLVS_RESULT_DIDNT_CONVERGE; break;
        case SolveResult::REDUNDANT_DIDNT_CONVERGE: sr.result = SLVS_RESULT_INCONSISTENT; break;
        case SolveResult::REDUNDANT_OKAY:          sr.result = SLVS_RESULT_REDUNDANT_OKAY; break;
        case SolveResult::TOO_MANY_UNKNOWNS:       sr.result = SLVS_RESULT_TOO_MANY_UNKNOWNS; break;
    }
    // Release the per-solve scratch heap. `System::Solve` allocates
    // Expr trees + Jacobian intermediates into the thread-local
    // `TempArena` mimalloc heap (see platform/platformbase.cpp).
    // Without this call the arena grows by ~hundreds of KB per
    // solve — invisible in test runs that only call once, but a
    // gigabyte-per-minute leak in a real-time tick loop. The legacy
    // one-shot `Slvs_SolveSketch3D` (below) already does this; the
    // incremental `Slvs_SolveSketch` had been missing it.
    Platform::FreeAllTemporary();
    return sr;
}

double Slvs_GetParamValue(Slvs_Solver *solver, uint32_t ph)
{
    WithCurrentSolver _ws(solver);
    Param* p = SK.param.FindById(hParam { ph });
    return p->val;
}

void Slvs_SetParamValue(Slvs_Solver *solver, uint32_t ph, double value)
{
    WithCurrentSolver _ws(solver);
    Param* p = SK.param.FindById(hParam { ph });
    p->val = value;
}

void Slvs_SetConstraintValue(Slvs_Solver *solver, uint32_t ch, double value)
{
    WithCurrentSolver _ws(solver);
    ConstraintBase* c = SK.constraint.FindById(hConstraint { ch });
    c->valA = value;
    // Also update the constraint's parameter if it exists
    if(c->valP.v) {
        Param* p = SK.param.FindById(c->valP);
        p->val = value;
    }
}

double Slvs_GetConstraintValue(Slvs_Solver *solver, uint32_t ch)
{
    WithCurrentSolver _ws(solver);
    ConstraintBase* c = SK.constraint.FindById(hConstraint { ch });
    return c->valA;
}

void Slvs_SetConstraintGroup(Slvs_Solver *solver, uint32_t ch, uint32_t new_group)
{
    WithCurrentSolver _ws(solver);
    // Move a constraint to a different group. Used by clients (e.g.
    // pyactiongraph's spatial-affector layer) that want to enable/disable
    // individual constraints at runtime: park them in an unused group to
    // exclude from solves, move back to the active group to re-enable.
    // The solver filters by exact `c->group == g->h` match in three
    // iteration sites (Slvs_SolveSketch param-collection, System::
    // WriteEquationsExceptFor, System::FindWhichToRemoveToFixJacobian),
    // so this single setter is sufficient.
    ConstraintBase* c = SK.constraint.FindById(hConstraint { ch });
    c->group.v = new_group;
}

uint32_t Slvs_GetConstraintGroup(Slvs_Solver *solver, uint32_t ch)
{
    WithCurrentSolver _ws(solver);
    ConstraintBase* c = SK.constraint.FindById(hConstraint { ch });
    return c->group.v;
}

void Slvs_Solve(Slvs_Solver *solver, Slvs_System *ssys, uint32_t shg)
{
    WithCurrentSolver _ws(solver);
    SYS.Clear();
    SK.param.Clear();
    SK.entity.Clear();
    SK.constraint.Clear();
    int i;
    for(i = 0; i < ssys->params; i++) {
        Slvs_Param *sp = &(ssys->param[i]);
        Param p = {};

        p.h.v = sp->h;
        p.val = sp->val;
        SK.param.Add(&p);
        if(sp->group == shg) {
            SYS.param.Add(&p);
        }
    }

    for(i = 0; i < ssys->entities; i++) {
        Slvs_Entity *se = &(ssys->entity[i]);
        EntityBase e = {};
        e.type = Slvs_CTypeToEntityBaseType(se->type);
        e.h.v           = se->h;
        e.group.v       = se->group;
        e.workplane.v   = se->wrkpl;
        e.point[0].v    = se->point[0];
        e.point[1].v    = se->point[1];
        e.point[2].v    = se->point[2];
        e.point[3].v    = se->point[3];
        e.normal.v      = se->normal;
        e.distance.v    = se->distance;
        e.param[0].v    = se->param[0];
        e.param[1].v    = se->param[1];
        e.param[2].v    = se->param[2];
        e.param[3].v    = se->param[3];

        SK.entity.Add(&e);
    }
    ParamList params = {};
    for(i = 0; i < ssys->constraints; i++) {
        Slvs_Constraint *sc = &(ssys->constraint[i]);
        ConstraintBase c = {};
        c.type = Slvs_CTypeToConstraintBaseType(sc->type);
        c.h.v           = sc->h;
        c.group.v       = sc->group;
        c.workplane.v   = sc->wrkpl;
        c.valA          = sc->valA;
        c.ptA.v         = sc->ptA;
        c.ptB.v         = sc->ptB;
        c.entityA.v     = sc->entityA;
        c.entityB.v     = sc->entityB;
        c.entityC.v     = sc->entityC;
        c.entityD.v     = sc->entityD;
        c.other         = (sc->other) ? true : false;
        c.other2        = (sc->other2) ? true : false;

        c.Generate(&params);
        if(!params.IsEmpty()) {
            for(Param &p : params) {
                p.h = SK.param.AddAndAssignId(&p);
                c.valP = p.h;
                SYS.param.Add(&p);
            }
            params.Clear();

            if(Slvs_CanInitiallySatisfy(c)) {
                c.ModifyToSatisfy();
            }
        }

        SK.constraint.Add(&c);
    }

    for(i = 0; i < ssys->ndragged; i++) {
        if(ssys->dragged[i]) {
            hParam hp = { ssys->dragged[i] };
            SYS.dragged.insert(hp);
        }
    }

    Group g = {};
    g.h.v = shg;

    List<hConstraint> bad = {};

    // Now we're finally ready to solve!
    bool andFindBad = ssys->calculateFaileds ? true : false;
    SolveResult how = SYS.Solve(&g, &(ssys->dof), &bad, andFindBad, /*andFindFree=*/false);

    switch(how) {
        case SolveResult::OKAY:
            ssys->result = SLVS_RESULT_OKAY;
            break;

        case SolveResult::DIDNT_CONVERGE:
            ssys->result = SLVS_RESULT_DIDNT_CONVERGE;
            break;

        case SolveResult::REDUNDANT_DIDNT_CONVERGE:
            ssys->result = SLVS_RESULT_INCONSISTENT;
            break;

        case SolveResult::REDUNDANT_OKAY:
            ssys->result = SLVS_RESULT_REDUNDANT_OKAY;
            break;

        case SolveResult::TOO_MANY_UNKNOWNS:
            ssys->result = SLVS_RESULT_TOO_MANY_UNKNOWNS;
            break;
    }

    // Write the new parameter values back to our caller.
    for(i = 0; i < ssys->params; i++) {
        Slvs_Param *sp = &(ssys->param[i]);
        hParam hp = { sp->h };
        sp->val = SK.GetParam(hp)->val;
    }

    if(ssys->failed) {
        // Copy over any the list of problematic constraints.
        for(i = 0; i < ssys->faileds && i < bad.n; i++) {
            ssys->failed[i] = bad[i].v;
        }
        ssys->faileds = bad.n;
    }

    bad.Clear();
    SYS.Clear();
    SK.param.Clear();
    SK.entity.Clear();
    SK.constraint.Clear();

    Platform::FreeAllTemporary();
}

// ---------- Multi-instance handle API (see slvs.h docs) ----------------------
//
// `Slvs_Solver` is exposed as an opaque struct; under the hood it's the
// `SolveSpace::Solver` C++ class. The four functions below are thin
// reinterpret_cast wrappers — Solver carries no extra state beyond what the
// C++ class already encapsulates (sketch, system, dragged set, temp arena).
//
// `Slvs_SetCurrentSolver` writes the C++-side `thread_local CurrentSolver`
// pointer. All other Slvs_* functions reach the per-solver state through
// the SK / SYS macros in solver.h, which in turn read `CurrentSolver`. So
// "selecting" a Solver for subsequent calls is a single pointer write.

Slvs_Solver *Slvs_CreateSolver(void) {
    return reinterpret_cast<Slvs_Solver *>(new SolveSpace::Solver());
}

void Slvs_DestroySolver(Slvs_Solver *handle) {
    if(handle == nullptr) return;
    auto *s = reinterpret_cast<SolveSpace::Solver *>(handle);
    // If the destroyed solver is currently selected, unselect it — the
    // next API call will lazy-allocate a fresh per-thread default. This
    // matches the "don't use after free" expectation without leaving a
    // dangling thread-local pointer.
    if(SolveSpace::CurrentSolver == s) {
        SolveSpace::CurrentSolver = nullptr;
    }
    delete s;
}

void Slvs_SetCurrentSolver(Slvs_Solver *handle) {
    SolveSpace::CurrentSolver = reinterpret_cast<SolveSpace::Solver *>(handle);
}

Slvs_Solver *Slvs_GetCurrentSolver(void) {
    return reinterpret_cast<Slvs_Solver *>(SolveSpace::CurrentSolver);
}

} /* extern "C" */
