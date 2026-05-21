#cython: language_level=3
from enum import IntEnum, auto
from libc.stdint cimport uint32_t
from libc.stdlib cimport free

cdef extern from "slvs.h" nogil:
    ctypedef uint32_t Slvs_hEntity
    ctypedef uint32_t Slvs_hGroup
    ctypedef uint32_t Slvs_hConstraint
    ctypedef uint32_t Slvs_hParam

    ctypedef struct Slvs_Entity:
        Slvs_hEntity h
        Slvs_hGroup group
        int type
        Slvs_hEntity wrkpl
        Slvs_hEntity point[4]
        Slvs_hEntity normal
        Slvs_hEntity distance
        Slvs_hParam param[4]

    ctypedef struct Slvs_Constraint:
        Slvs_hConstraint h
        Slvs_hGroup group
        int type
        Slvs_hEntity wrkpl
        double valA
        Slvs_hEntity ptA
        Slvs_hEntity ptB
        Slvs_hEntity entityA
        Slvs_hEntity entityB
        Slvs_hEntity entityC
        Slvs_hEntity entityD
        int other
        int other2

    ctypedef struct Slvs_SolveResult:
        int result
        int dof
        int nbad

    # Pure-math helpers — no Solver state involved.
    void Slvs_QuaternionU(double qw, double qx, double qy, double qz,
                             double *x, double *y, double *z)
    void Slvs_QuaternionV(double qw, double qx, double qy, double qz,
                             double *x, double *y, double *z)
    void Slvs_QuaternionN(double qw, double qx, double qy, double qz,
                             double *x, double *y, double *z)
    void Slvs_MakeQuaternion(double ux, double uy, double uz,
                             double vx, double vy, double vz,
                             double *qw, double *qx, double *qy, double *qz)

    # Multi-instance solver handle (opaque). Every data-mutating Slvs_*
    # function takes a `Slvs_Solver *` as its first argument.
    ctypedef struct Slvs_Solver:
        pass
    Slvs_Solver *Slvs_CreateSolver()
    void         Slvs_DestroySolver(Slvs_Solver *solver)

    Slvs_Entity Slvs_AddPoint2D(Slvs_Solver *solver, Slvs_hGroup grouph, double u, double v, Slvs_Entity workplane)
    Slvs_Entity Slvs_AddPoint3D(Slvs_Solver *solver, Slvs_hGroup grouph, double x, double y, double z)
    Slvs_Entity Slvs_AddNormal2D(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity workplane)
    Slvs_Entity Slvs_AddNormal3D(Slvs_Solver *solver, Slvs_hGroup grouph, double qw, double qx, double qy, double qz)
    Slvs_Entity Slvs_AddDistance(Slvs_Solver *solver, Slvs_hGroup grouph, double value, Slvs_Entity workplane)
    Slvs_Entity Slvs_AddLine2D(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane)
    Slvs_Entity Slvs_AddLine3D(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity ptB)
    Slvs_Entity Slvs_AddCubic(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity ptC, Slvs_Entity ptD, Slvs_Entity workplane)
    Slvs_Entity Slvs_AddArc(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity normal, Slvs_Entity center, Slvs_Entity start, Slvs_Entity end, Slvs_Entity workplane)
    Slvs_Entity Slvs_AddCircle(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity normal, Slvs_Entity center, Slvs_Entity radius, Slvs_Entity workplane)
    Slvs_Entity Slvs_AddWorkplane(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity origin, Slvs_Entity nm)
    Slvs_Entity Slvs_AddBase2D(Slvs_Solver *solver, Slvs_hGroup grouph)

    Slvs_Constraint Slvs_AddConstraint(Slvs_Solver *solver, Slvs_hGroup grouph, int type, Slvs_Entity workplane, double val, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity entityC, Slvs_Entity entityD, int other, int other2)
    Slvs_Constraint Slvs_Coincident(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Distance(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Equal(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane)
    Slvs_Constraint Slvs_EqualAngle(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity entityC, Slvs_Entity entityD, Slvs_Entity workplane)
    Slvs_Constraint Slvs_EqualPointToLine(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity entityC, Slvs_Entity entityD, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Ratio(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Symmetric(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity entityC, Slvs_Entity workplane)
    Slvs_Constraint Slvs_SymmetricH(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane)
    Slvs_Constraint Slvs_SymmetricV(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Midpoint(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity ptB, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Horizontal(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity workplane, Slvs_Entity entityB)
    Slvs_Constraint Slvs_Vertical(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity workplane, Slvs_Entity entityB)
    Slvs_Constraint Slvs_Diameter(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, double value)
    Slvs_Constraint Slvs_SameOrientation(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB)
    Slvs_Constraint Slvs_Angle(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane, int inverse)
    Slvs_Constraint Slvs_Perpendicular(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane, int inverse)
    Slvs_Constraint Slvs_Parallel(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Tangent(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, Slvs_Entity workplane)
    Slvs_Constraint Slvs_DistanceProj(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity ptB, double value)
    Slvs_Constraint Slvs_LengthDiff(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity entityA, Slvs_Entity entityB, double value, Slvs_Entity workplane)
    Slvs_Constraint Slvs_Dragged(Slvs_Solver *solver, Slvs_hGroup grouph, Slvs_Entity ptA, Slvs_Entity workplane)

    void Slvs_MarkDragged(Slvs_Solver *solver, Slvs_Entity ptA)
    Slvs_SolveResult Slvs_SolveSketch(Slvs_Solver *solver, Slvs_hGroup hg, Slvs_hConstraint **bad) nogil
    double Slvs_GetParamValue(Slvs_Solver *solver, int ph)
    double Slvs_SetParamValue(Slvs_Solver *solver, int ph, double value)
    double Slvs_GetConstraintValue(Slvs_Solver *solver, int ch)
    void Slvs_SetConstraintValue(Slvs_Solver *solver, int ch, double value)
    uint32_t Slvs_GetConstraintGroup(Slvs_Solver *solver, uint32_t ch)
    void Slvs_SetConstraintGroup(Slvs_Solver *solver, uint32_t ch, uint32_t new_group)
    void Slvs_ClearSketch(Slvs_Solver *solver)

    cdef Slvs_Entity _E_NONE "SLVS_E_NONE"
    cdef Slvs_Entity _E_FREE_IN_3D "SLVS_E_FREE_IN_3D"

    cdef int _SLVS_C_POINTS_COINCIDENT "SLVS_C_POINTS_COINCIDENT"
    cdef int _SLVS_C_PT_PT_DISTANCE "SLVS_C_PT_PT_DISTANCE"
    cdef int _SLVS_C_PT_PLANE_DISTANCE "SLVS_C_PT_PLANE_DISTANCE"
    cdef int _SLVS_C_PT_LINE_DISTANCE "SLVS_C_PT_LINE_DISTANCE"
    cdef int _SLVS_C_PT_FACE_DISTANCE "SLVS_C_PT_FACE_DISTANCE"
    cdef int _SLVS_C_PT_IN_PLANE "SLVS_C_PT_IN_PLANE"
    cdef int _SLVS_C_PT_ON_LINE "SLVS_C_PT_ON_LINE"
    cdef int _SLVS_C_PT_ON_FACE "SLVS_C_PT_ON_FACE"
    cdef int _SLVS_C_EQUAL_LENGTH_LINES "SLVS_C_EQUAL_LENGTH_LINES"
    cdef int _SLVS_C_LENGTH_RATIO "SLVS_C_LENGTH_RATIO"
    cdef int _SLVS_C_EQ_LEN_PT_LINE_D "SLVS_C_EQ_LEN_PT_LINE_D"
    cdef int _SLVS_C_EQ_PT_LN_DISTANCES "SLVS_C_EQ_PT_LN_DISTANCES"
    cdef int _SLVS_C_EQUAL_ANGLE "SLVS_C_EQUAL_ANGLE"
    cdef int _SLVS_C_EQUAL_LINE_ARC_LEN "SLVS_C_EQUAL_LINE_ARC_LEN"
    cdef int _SLVS_C_SYMMETRIC "SLVS_C_SYMMETRIC"
    cdef int _SLVS_C_SYMMETRIC_HORIZ "SLVS_C_SYMMETRIC_HORIZ"
    cdef int _SLVS_C_SYMMETRIC_VERT "SLVS_C_SYMMETRIC_VERT"
    cdef int _SLVS_C_SYMMETRIC_LINE "SLVS_C_SYMMETRIC_LINE"
    cdef int _SLVS_C_AT_MIDPOINT "SLVS_C_AT_MIDPOINT"
    cdef int _SLVS_C_HORIZONTAL "SLVS_C_HORIZONTAL"
    cdef int _SLVS_C_VERTICAL "SLVS_C_VERTICAL"
    cdef int _SLVS_C_DIAMETER "SLVS_C_DIAMETER"
    cdef int _SLVS_C_PT_ON_CIRCLE "SLVS_C_PT_ON_CIRCLE"
    cdef int _SLVS_C_SAME_ORIENTATION "SLVS_C_SAME_ORIENTATION"
    cdef int _SLVS_C_ANGLE "SLVS_C_ANGLE"
    cdef int _SLVS_C_PARALLEL "SLVS_C_PARALLEL"
    cdef int _SLVS_C_PERPENDICULAR "SLVS_C_PERPENDICULAR"
    cdef int _SLVS_C_ARC_LINE_TANGENT "SLVS_C_ARC_LINE_TANGENT"
    cdef int _SLVS_C_CUBIC_LINE_TANGENT "SLVS_C_CUBIC_LINE_TANGENT"
    cdef int _SLVS_C_EQUAL_RADIUS "SLVS_C_EQUAL_RADIUS"
    cdef int _SLVS_C_PROJ_PT_DISTANCE "SLVS_C_PROJ_PT_DISTANCE"
    cdef int _SLVS_C_WHERE_DRAGGED "SLVS_C_WHERE_DRAGGED"
    cdef int _SLVS_C_CURVE_CURVE_TANGENT "SLVS_C_CURVE_CURVE_TANGENT"
    cdef int _SLVS_C_LENGTH_DIFFERENCE "SLVS_C_LENGTH_DIFFERENCE"
    cdef int _SLVS_C_ARC_ARC_LEN_RATIO "SLVS_C_ARC_ARC_LEN_RATIO"
    cdef int _SLVS_C_ARC_LINE_LEN_RATIO "SLVS_C_ARC_LINE_LEN_RATIO"
    cdef int _SLVS_C_ARC_ARC_DIFFERENCE "SLVS_C_ARC_ARC_DIFFERENCE"
    cdef int _SLVS_C_ARC_LINE_DIFFERENCE "SLVS_C_ARC_LINE_DIFFERENCE"

    cdef int _SLVS_E_POINT_IN_3D "SLVS_E_POINT_IN_3D"
    cdef int _SLVS_E_POINT_IN_2D "SLVS_E_POINT_IN_2D"
    cdef int _SLVS_E_NORMAL_IN_3D "SLVS_E_NORMAL_IN_3D"
    cdef int _SLVS_E_NORMAL_IN_2D "SLVS_E_NORMAL_IN_2D"
    cdef int _SLVS_E_DISTANCE "SLVS_E_DISTANCE"
    cdef int _SLVS_E_WORKPLANE "SLVS_E_WORKPLANE"
    cdef int _SLVS_E_LINE_SEGMENT "SLVS_E_LINE_SEGMENT"
    cdef int _SLVS_E_CUBIC "SLVS_E_CUBIC"
    cdef int _SLVS_E_CIRCLE "SLVS_E_CIRCLE"
    cdef int _SLVS_E_ARC_OF_CIRCLE "SLVS_E_ARC_OF_CIRCLE"

    cdef int _SLVS_RESULT_OKAY "SLVS_RESULT_OKAY"
    cdef int _SLVS_RESULT_INCONSISTENT "SLVS_RESULT_INCONSISTENT"
    cdef int _SLVS_RESULT_DIDNT_CONVERGE "SLVS_RESULT_DIDNT_CONVERGE"
    cdef int _SLVS_RESULT_TOO_MANY_UNKNOWNS "SLVS_RESULT_TOO_MANY_UNKNOWNS"
    cdef int _SLVS_RESULT_REDUNDANT_OKAY "SLVS_RESULT_REDUNDANT_OKAY"

E_NONE = _E_NONE
E_FREE_IN_3D = _E_FREE_IN_3D


# Pure-math helpers (no Solver state) stay module-level.
cpdef tuple quaternion_u(double qw, double qx, double qy, double qz):
    """Input quaternion, return unit vector of U axis."""
    cdef double x, y, z
    Slvs_QuaternionU(qw, qx, qy, qz, &x, &y, &z)
    return x, y, z


cpdef tuple quaternion_v(double qw, double qx, double qy, double qz):
    """Input quaternion, return unit vector of V axis."""
    cdef double x, y, z
    Slvs_QuaternionV(qw, qx, qy, qz, &x, &y, &z)
    return x, y, z


cpdef tuple quaternion_n(double qw, double qx, double qy, double qz):
    """Input quaternion, return unit vector of normal."""
    cdef double x, y, z
    Slvs_QuaternionN(qw, qx, qy, qz, &x, &y, &z)
    return x, y, z


cpdef tuple make_quaternion(double ux, double uy, double uz, double vx, double vy, double vz):
    """Input two unit vectors, return quaternion."""
    cdef double qw, qx, qy, qz
    Slvs_MakeQuaternion(ux, uy, uz, vx, vy, vz, &qw, &qx, &qy, &qz)
    return qw, qx, qy, qz


# Enums.
class ResultFlag(IntEnum):
    """Symbol of the result flags."""
    OKAY = _SLVS_RESULT_OKAY
    INCONSISTENT = _SLVS_RESULT_INCONSISTENT
    DIDNT_CONVERGE = _SLVS_RESULT_DIDNT_CONVERGE
    TOO_MANY_UNKNOWNS = _SLVS_RESULT_TOO_MANY_UNKNOWNS
    REDUNDANT_OKAY = _SLVS_RESULT_REDUNDANT_OKAY


class ConstraintType(IntEnum):
    """Symbol of the constraint types."""
    POINTS_COINCIDENT = _SLVS_C_POINTS_COINCIDENT
    PT_PT_DISTANCE = _SLVS_C_PT_PT_DISTANCE
    PT_PLANE_DISTANCE = _SLVS_C_PT_PLANE_DISTANCE
    PT_LINE_DISTANCE = _SLVS_C_PT_LINE_DISTANCE
    PT_FACE_DISTANCE = _SLVS_C_PT_FACE_DISTANCE
    PT_IN_PLANE = _SLVS_C_PT_IN_PLANE
    PT_ON_LINE = _SLVS_C_PT_ON_LINE
    PT_ON_FACE = _SLVS_C_PT_ON_FACE
    EQUAL_LENGTH_LINES = _SLVS_C_EQUAL_LENGTH_LINES
    LENGTH_RATIO = _SLVS_C_LENGTH_RATIO
    EQ_LEN_PT_LINE_D = _SLVS_C_EQ_LEN_PT_LINE_D
    EQ_PT_LN_DISTANCES = _SLVS_C_EQ_PT_LN_DISTANCES
    EQUAL_ANGLE = _SLVS_C_EQUAL_ANGLE
    EQUAL_LINE_ARC_LEN = _SLVS_C_EQUAL_LINE_ARC_LEN
    SYMMETRIC = _SLVS_C_SYMMETRIC
    SYMMETRIC_HORIZ = _SLVS_C_SYMMETRIC_HORIZ
    SYMMETRIC_VERT = _SLVS_C_SYMMETRIC_VERT
    SYMMETRIC_LINE = _SLVS_C_SYMMETRIC_LINE
    AT_MIDPOINT = _SLVS_C_AT_MIDPOINT
    HORIZONTAL = _SLVS_C_HORIZONTAL
    VERTICAL = _SLVS_C_VERTICAL
    DIAMETER = _SLVS_C_DIAMETER
    PT_ON_CIRCLE = _SLVS_C_PT_ON_CIRCLE
    SAME_ORIENTATION = _SLVS_C_SAME_ORIENTATION
    ANGLE = _SLVS_C_ANGLE
    PARALLEL = _SLVS_C_PARALLEL
    PERPENDICULAR = _SLVS_C_PERPENDICULAR
    ARC_LINE_TANGENT = _SLVS_C_ARC_LINE_TANGENT
    CUBIC_LINE_TANGENT = _SLVS_C_CUBIC_LINE_TANGENT
    EQUAL_RADIUS = _SLVS_C_EQUAL_RADIUS
    PROJ_PT_DISTANCE = _SLVS_C_PROJ_PT_DISTANCE
    WHERE_DRAGGED = _SLVS_C_WHERE_DRAGGED
    CURVE_CURVE_TANGENT = _SLVS_C_CURVE_CURVE_TANGENT
    LENGTH_DIFFERENCE = _SLVS_C_LENGTH_DIFFERENCE
    ARC_ARC_LEN_RATIO = _SLVS_C_ARC_ARC_LEN_RATIO
    ARC_LINE_LEN_RATIO = _SLVS_C_ARC_LINE_LEN_RATIO
    ARC_ARC_DIFFERENCE = _SLVS_C_ARC_ARC_DIFFERENCE
    ARC_LINE_DIFFERENCE = _SLVS_C_ARC_LINE_DIFFERENCE


class EntityType(IntEnum):
    POINT_IN_3D = _SLVS_E_POINT_IN_3D
    POINT_IN_2D = _SLVS_E_POINT_IN_2D
    NORMAL_IN_3D = _SLVS_E_NORMAL_IN_3D
    NORMAL_IN_2D = _SLVS_E_NORMAL_IN_2D
    DISTANCE = _SLVS_E_DISTANCE
    WORKPLANE = _SLVS_E_WORKPLANE
    LINE_SEGMENT = _SLVS_E_LINE_SEGMENT
    CUBIC = _SLVS_E_CUBIC
    CIRCLE = _SLVS_E_CIRCLE
    ARC_OF_CIRCLE = _SLVS_E_ARC_OF_CIRCLE


# `Solver` is the only sketch/solve API surface. Each instance owns one
# underlying `Slvs_Solver *` (independent sketch, system, dragged set,
# scratch arena). Multiple instances are fully independent — share none
# of solver state — and may be driven concurrently on different threads.
#
# Every method passes `self.handle` through to its `Slvs_*` C entry
# point, so the choice of which solver is mutated is explicit at every
# call site.
cdef class Solver:
    cdef Slvs_Solver *handle

    def __cinit__(self):
        self.handle = Slvs_CreateSolver()
        if self.handle is NULL:
            raise MemoryError("Slvs_CreateSolver failed")

    def __dealloc__(self):
        if self.handle is not NULL:
            Slvs_DestroySolver(self.handle)
            self.handle = NULL

    # ---------- entities ----------
    def add_point_2d(self, grouph: int, u: float, v: float, workplane: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddPoint2D(self.handle, grouph, u, v, workplane)

    def add_point_3d(self, grouph: int, x: float, y: float, z: float) -> Slvs_Entity:
        return Slvs_AddPoint3D(self.handle, grouph, x, y, z)

    def add_normal_2d(self, grouph: int, workplane: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddNormal2D(self.handle, grouph, workplane)

    def add_normal_3d(self, grouph: int, qw: float, qx: float, qy: float, qz: float) -> Slvs_Entity:
        return Slvs_AddNormal3D(self.handle, grouph, qw, qx, qy, qz)

    def add_distance(self, grouph: int, value: float, workplane: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddDistance(self.handle, grouph, value, workplane)

    def add_line_2d(self, grouph: int, ptA: Slvs_Entity, ptB: Slvs_Entity, workplane: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddLine2D(self.handle, grouph, ptA, ptB, workplane)

    def add_line_3d(self, grouph: int, ptA: Slvs_Entity, ptB: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddLine3D(self.handle, grouph, ptA, ptB)

    def add_cubic(self, grouph: int, ptA: Slvs_Entity, ptB: Slvs_Entity, ptC: Slvs_Entity, ptD: Slvs_Entity, workplane: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddCubic(self.handle, grouph, ptA, ptB, ptC, ptD, workplane)

    def add_arc(self, grouph: int, normal: Slvs_Entity, center: Slvs_Entity, start: Slvs_Entity, end: Slvs_Entity, workplane: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddArc(self.handle, grouph, normal, center, start, end, workplane)

    def add_circle(self, grouph: int, normal: Slvs_Entity, center: Slvs_Entity, radius: Slvs_Entity, workplane: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddCircle(self.handle, grouph, normal, center, radius, workplane)

    def add_workplane(self, grouph: int, origin: Slvs_Entity, nm: Slvs_Entity) -> Slvs_Entity:
        return Slvs_AddWorkplane(self.handle, grouph, origin, nm)

    def add_base_2d(self, grouph: int) -> Slvs_Entity:
        return Slvs_AddBase2D(self.handle, grouph)

    # ---------- constraints ----------
    def add_constraint(self, grouph: int, c_type, workplane: Slvs_Entity, val: float, ptA: Slvs_Entity = E_NONE,
            ptB: Slvs_Entity = E_NONE, entityA: Slvs_Entity = E_NONE,
            entityB: Slvs_Entity = E_NONE, entityC: Slvs_Entity = E_NONE,
            entityD: Slvs_Entity = E_NONE, other: int = 0, other2: int = 0) -> Slvs_Constraint:
        # `c_type` is intentionally untyped: callers commonly pass `ConstraintType`
        # (IntEnum), which Cython 3.2+ refuses to coerce to a typed `int` parameter
        # even though `IntEnum` is `int`-subclass. Coerce explicitly here.
        return Slvs_AddConstraint(self.handle, grouph, int(c_type), workplane, val, ptA, ptB, entityA, entityB, entityC, entityD, other, other2)

    def coincident(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Coincident(self.handle, grouph, entityA, entityB, workplane)

    def distance(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, value: float, workplane: Slvs_Entity) -> Slvs_Constraint:
        return Slvs_Distance(self.handle, grouph, entityA, entityB, value, workplane)

    def equal(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Equal(self.handle, grouph, entityA, entityB, workplane)

    def equal_angle(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, entityC: Slvs_Entity,
                                            entityD: Slvs_Entity,
                                            workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_EqualAngle(self.handle, grouph, entityA, entityB, entityC, entityD, workplane)

    def equal_point_to_line(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity,
                                            entityC: Slvs_Entity, entityD: Slvs_Entity,
                                            workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_EqualPointToLine(self.handle, grouph, entityA, entityB, entityC, entityD, workplane)

    def ratio(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, value: float, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Ratio(self.handle, grouph, entityA, entityB, value, workplane)

    def symmetric(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, entityC: Slvs_Entity = E_NONE, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Symmetric(self.handle, grouph, entityA, entityB, entityC, workplane)

    def symmetric_h(self, grouph: int, ptA: Slvs_Entity, ptB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_SymmetricH(self.handle, grouph, ptA, ptB, workplane)

    def symmetric_v(self, grouph: int, ptA: Slvs_Entity, ptB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_SymmetricV(self.handle, grouph, ptA, ptB, workplane)

    def midpoint(self, grouph: int, ptA: Slvs_Entity, ptB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Midpoint(self.handle, grouph, ptA, ptB, workplane)

    def horizontal(self, grouph: int, entityA: Slvs_Entity, workplane: Slvs_Entity, entityB: Slvs_Entity = E_NONE) -> Slvs_Constraint:
        return Slvs_Horizontal(self.handle, grouph, entityA, workplane, entityB)

    def vertical(self, grouph: int, entityA: Slvs_Entity, workplane: Slvs_Entity, entityB: Slvs_Entity = E_NONE) -> Slvs_Constraint:
        return Slvs_Vertical(self.handle, grouph, entityA, workplane, entityB)

    def diameter(self, grouph: int, entityA: Slvs_Entity, value: float) -> Slvs_Constraint:
        return Slvs_Diameter(self.handle, grouph, entityA, value)

    def same_orientation(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity) -> Slvs_Constraint:
        return Slvs_SameOrientation(self.handle, grouph, entityA, entityB)

    def angle(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, value: float, workplane: Slvs_Entity = E_FREE_IN_3D, inverse: bool = False) -> Slvs_Constraint:
        return Slvs_Angle(self.handle, grouph, entityA, entityB, value, workplane, inverse)

    def perpendicular(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D, inverse: bool = False) -> Slvs_Constraint:
        return Slvs_Perpendicular(self.handle, grouph, entityA, entityB, workplane, inverse)

    def parallel(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Parallel(self.handle, grouph, entityA, entityB, workplane)

    def tangent(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Tangent(self.handle, grouph, entityA, entityB, workplane)

    def distance_proj(self, grouph: int, ptA: Slvs_Entity, ptB: Slvs_Entity, value: float) -> Slvs_Constraint:
        return Slvs_DistanceProj(self.handle, grouph, ptA, ptB, value)

    def length_diff(self, grouph: int, entityA: Slvs_Entity, entityB: Slvs_Entity, value: float, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_LengthDiff(self.handle, grouph, entityA, entityB, value, workplane)

    def dragged(self, grouph: int, ptA: Slvs_Entity, workplane: Slvs_Entity = E_FREE_IN_3D) -> Slvs_Constraint:
        return Slvs_Dragged(self.handle, grouph, ptA, workplane)

    # ---------- params / constraint values / solving ----------
    def mark_dragged(self, ptA: Slvs_Entity):
        Slvs_MarkDragged(self.handle, ptA)

    def solve_sketch(self, grouph: int, calculateFaileds: bool):
        # Release the GIL during the solve so other Python threads (notably the
        # asyncio event loop in a parallel coroutine) can run. The underlying
        # `Slvs_SolveSketch` is declared `nogil` in `slvs.h` and touches no
        # Python objects; without `with nogil:` here, the GIL would be held for
        # the duration of the solve and starve every other coroutine — exactly
        # the bug that left the viz server stuck at ~7 broadcasts/sec when the
        # delta robot's solve took ~45 ms per tick.
        cdef Slvs_hConstraint *badp = NULL
        cdef Slvs_hGroup hg = grouph
        cdef Slvs_SolveResult result
        cdef Slvs_Solver *h = self.handle
        if not calculateFaileds:
            with nogil:
                result = Slvs_SolveSketch(h, hg, NULL)
            return result
        else:
            with nogil:
                result = Slvs_SolveSketch(h, hg, &badp)
            bad = []
            if badp != NULL:
                for i in range(0, result.nbad):
                    bad.append(badp[i])
                free(badp)
            return result, bad

    def get_param_value(self, ph: int):
        return Slvs_GetParamValue(self.handle, ph)

    def set_param_value(self, ph: int, value: float):
        Slvs_SetParamValue(self.handle, ph, value)

    def get_constraint_value(self, ch: int):
        return Slvs_GetConstraintValue(self.handle, ch)

    def set_constraint_value(self, ch: int, value: float):
        Slvs_SetConstraintValue(self.handle, ch, value)

    def get_constraint_group(self, ch: int) -> int:
        return Slvs_GetConstraintGroup(self.handle, ch)

    def set_constraint_group(self, ch: int, new_group: int):
        """Move a constraint to a different group.

        Used to enable/disable constraints at runtime: parking a constraint
        in an unused group excludes it from the solve; moving it back to
        an active group re-enables it. The solver filters constraints by
        exact group match in three iteration sites; this single setter
        flips participation for all of them.
        """
        Slvs_SetConstraintGroup(self.handle, ch, new_group)

    def clear_sketch(self):
        Slvs_ClearSketch(self.handle)
