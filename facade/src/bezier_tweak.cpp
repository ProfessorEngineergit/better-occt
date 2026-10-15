// See bezier_tweak.h for what this is and why it is built this way.

#include "bezier_tweak.h"

#include <BRepCheck.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepLib.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI_IntCS.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomProjLib.hxx>
#include <Geom_BoundedCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

/// A face's surface without the trimming a face may store it with, so it reaches past the face.
occ::handle<Geom_Surface> unbounded(occ::handle<Geom_Surface> surface) {
    while (!surface.IsNull()) {
        auto trimmed = occ::handle<Geom_RectangularTrimmedSurface>::DownCast(surface);
        if (trimmed.IsNull())
            break;
        surface = trimmed->BasisSurface();
    }
    return surface;
}

/// The same for a curve: a corner that moves along an edge may leave the stretch it was cut to.
occ::handle<Geom_Curve> unbounded(occ::handle<Geom_Curve> curve) {
    while (!curve.IsNull()) {
        auto trimmed = occ::handle<Geom_TrimmedCurve>::DownCast(curve);
        if (trimmed.IsNull())
            break;
        curve = trimmed->BasisCurve();
    }
    return curve;
}

/// A pcurve that is only a stretch of a line or conic is handed back whole, so the edge's new
/// range never falls off its end. A spline keeps its stretch: it is defined only there.
occ::handle<Geom2d_Curve> unbounded(occ::handle<Geom2d_Curve> curve) {
    while (!curve.IsNull()) {
        auto trimmed = occ::handle<Geom2d_TrimmedCurve>::DownCast(curve);
        if (trimmed.IsNull())
            break;
        const auto basis = trimmed->BasisCurve();
        if (basis->IsKind(STANDARD_TYPE(Geom2d_BSplineCurve)) ||
            basis->IsKind(STANDARD_TYPE(Geom2d_BezierCurve)))
            break;
        curve = basis;
    }
    return curve;
}

/// The point of a curve nearest to `near`, and its parameter.
bool nearestOn(const occ::handle<Geom_Curve>& curve, const gp_Pnt& near, double& param,
               double& distance) {
    GeomAPI_ProjectPointOnCurve projection(near, curve);
    if (projection.NbPoints() == 0)
        return false;
    param = projection.LowerDistanceParameter();
    distance = projection.LowerDistance();
    return true;
}

gp_Pnt middleOf(const occ::handle<Geom_Curve>& curve, double first, double last) {
    return curve->Value(0.5 * (first + last));
}

/// Whether a stretch of a curve lies on a surface, checked at a few points along it.
bool liesOn(const occ::handle<Geom_Curve>& curve, double first, double last,
            const occ::handle<Geom_Surface>& surface) {
    for (int i = 0; i <= 4; ++i) {
        const gp_Pnt point = curve->Value(first + (last - first) * i / 4.0);
        GeomAPI_ProjectPointOnSurf onSurface(point, surface);
        if (onSurface.NbPoints() == 0 || onSurface.LowerDistance() > Precision::Confusion())
            return false;
    }
    return true;
}

/// The nearest whole number of periods from `from` to `to`, in parameter units.
double periodsBetween(double from, double to, double period) {
    return std::round((to - from) / period) * period;
}

} // namespace

BezierTweakModification::BezierTweakModification(const TopoDS_Shape& shape,
                                                 const std::vector<BezierFaceMove>& moves)
    : myMoves(moves) {
    try {
        prepare(shape);
    } catch (const Standard_Failure& failure) {
        throw std::runtime_error(std::string("tweak (prepare): ") + failure.ExceptionType() + " " +
                                 failure.what());
    }
}

void BezierTweakModification::prepare(const TopoDS_Shape& shape) {
    for (size_t i = 0; i < myMoves.size(); ++i) {
        const auto& face = myMoves[i].face;
        if (face.IsNull() || face.ShapeType() != TopAbs_FACE)
            throw std::runtime_error("tweak: only faces can be moved");
        myFaceMove.Bind(face, static_cast<int>(i));
    }
    TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_EDGE, TopAbs_FACE, myEdgeFaces);
    TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_VERTEX, TopAbs_FACE, myVertexFaces);
    TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, myVertexEdges);

    // New surfaces: the face's own, carried by its motion, then offset outward.
    for (const auto& move : myMoves) {
        const TopoDS_Face& face = TopoDS::Face(move.face);
        auto surface = unbounded(BRep_Tool::Surface(face));
        if (surface.IsNull())
            throw std::runtime_error("tweak: a face has no surface");
        auto moved = occ::handle<Geom_Surface>::DownCast(surface->Transformed(move.motion));
        if (std::abs(move.offset) > Precision::Confusion()) {
            // Outward is the surface's normal, turned round where the face is reversed.
            const double sense = face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
            occ::handle<Geom_OffsetSurface> offset =
                new Geom_OffsetSurface(moved, move.offset * sense);
            // A canonical offset (a cylinder of another radius) keeps the parameters of the
            // surface it came from, which the pcurve matching below relies on.
            auto plain = offset->Surface();
            moved = plain.IsNull() ? occ::handle<Geom_Surface>(offset) : plain;
        }
        mySurfaces.Bind(move.face, moved);
    }

    // New edges.
    for (int i = 1; i <= myEdgeFaces.Extent(); ++i) {
        const TopoDS_Edge& edge = TopoDS::Edge(myEdgeFaces.FindKey(i));
        if (BRep_Tool::Degenerated(edge))
            continue;
        const auto& faces = myEdgeFaces.FindFromIndex(i);
        int movedCount = 0;
        int commonMove = -2;
        int anyMove = -1;
        for (const auto& f : faces) {
            const int m = moveOf(f);
            if (m < 0)
                continue;
            ++movedCount;
            if (anyMove < 0)
                anyMove = m;
            commonMove = commonMove == -2 ? m : (commonMove == m ? m : -1);
        }
        if (movedCount == 0)
            continue;
        double first = 0.0;
        double last = 0.0;
        auto old = unbounded(BRep_Tool::Curve(edge, first, last));
        if (old.IsNull())
            continue;
        const gp_Trsf& motion = myMoves[static_cast<size_t>(anyMove)].motion;
        // Every face it lies on moves together and keeps its size: the edge is simply carried.
        if (movedCount == faces.Extent() && commonMove >= 0 &&
            std::abs(myMoves[static_cast<size_t>(commonMove)].offset) <= Precision::Confusion()) {
            myCurves.Bind(edge, occ::handle<Geom_Curve>::DownCast(old->Transformed(motion)));
            continue;
        }
        if (faces.Extent() != 2) {
            // A seam: the face meets itself. It moves with the face; an offset moves it outward.
            const TopoDS_Face& face = TopoDS::Face(faces.First());
            auto carriedCurve = occ::handle<Geom_Curve>::DownCast(old->Transformed(motion));
            const double offset = myMoves[static_cast<size_t>(anyMove)].offset;
            if (std::abs(offset) > Precision::Confusion()) {
                // An offset surface keeps its parameters, so the seam is the same iso-line on it.
                auto carriedSurface = occ::handle<Geom_Surface>::DownCast(
                    unbounded(BRep_Tool::Surface(face))->Transformed(motion));
                GeomAPI_ProjectPointOnSurf onSurface(middleOf(carriedCurve, first, last),
                                                     carriedSurface);
                if (onSurface.NbPoints() == 0)
                    throw std::runtime_error("tweak: a seam could not be found on its face");
                double u = 0.0;
                double v = 0.0;
                onSurface.LowerDistanceParameters(u, v);
                const gp_Pnt probe = carriedCurve->Value(first + 0.25 * (last - first));
                double param = 0.0;
                double alongU = 0.0;
                nearestOn(carriedSurface->UIso(u), probe, param, alongU);
                auto after = surfaceAfter(face);
                auto seam = alongU < 1e-6 ? after->UIso(u) : after->VIso(v);
                gp_Pnt p0;
                gp_Vec t0;
                carriedCurve->D1(0.5 * (first + last), p0, t0);
                double at = 0.0;
                double gap = 0.0;
                nearestOn(seam, p0, at, gap);
                gp_Pnt p1;
                gp_Vec t1;
                seam->D1(at, p1, t1);
                myCurves.Bind(edge, t1.Dot(t0) < 0.0 ? seam->Reversed() : seam);
                continue;
            }
            myCurves.Bind(edge, carriedCurve);
            continue;
        }
        // Carried by one face's motion, the edge may still lie on both faces: a face slid along
        // the plane of its neighbour, or a rounding carried with the face it rounds while the
        // wall it meets tangentially runs the way it moved. Kept as it is then. Two surfaces that
        // touch along a line are where intersecting them is least reliable.
        const TopoDS_Face& a = TopoDS::Face(faces.First());
        const TopoDS_Face& b = TopoDS::Face(faces.Last());
        occ::handle<Geom_Curve> kept;
        for (const auto& face : {a, b}) {
            const int m = moveOf(face);
            if (m < 0 || !kept.IsNull())
                continue;
            auto carriedCurve = occ::handle<Geom_Curve>::DownCast(
                old->Transformed(myMoves[static_cast<size_t>(m)].motion));
            if (liesOn(carriedCurve, first, last, surfaceAfter(a)) &&
                liesOn(carriedCurve, first, last, surfaceAfter(b)))
                kept = carriedCurve;
        }
        if (!kept.IsNull()) {
            myCurves.Bind(edge, kept);
            continue;
        }
        gp_Pnt p0;
        gp_Vec tangent;
        old->D1(0.5 * (first + last), p0, tangent);
        myCurves.Bind(edge, meetingOf(a, b, p0, tangent));
    }

    // New vertices.
    for (int i = 1; i <= myVertexFaces.Extent(); ++i) {
        const TopoDS_Vertex& vertex = TopoDS::Vertex(myVertexFaces.FindKey(i));
        const auto& faces = myVertexFaces.FindFromIndex(i);
        int movedCount = 0;
        int commonMove = -2;
        int firstMove = -1;
        for (const auto& f : faces) {
            const int m = moveOf(f);
            if (m < 0)
                continue;
            ++movedCount;
            if (firstMove < 0)
                firstMove = m;
            commonMove = commonMove == -2 ? m : (commonMove == m ? m : -1);
        }
        if (movedCount == 0)
            continue;
        const gp_Pnt old = BRep_Tool::Pnt(vertex);
        if (movedCount == faces.Extent() && commonMove >= 0 &&
            std::abs(myMoves[static_cast<size_t>(commonMove)].offset) <= Precision::Confusion()) {
            myPoints.Bind(vertex, old.Transformed(myMoves[static_cast<size_t>(commonMove)].motion));
            continue;
        }
        const gp_Pnt reference = old.Transformed(myMoves[static_cast<size_t>(firstMove)].motion);
        // An edge from the vertex, and a face round it that the edge does not lie on: where the
        // edge's new curve meets that face's new surface is the corner.
        bool placed = false;
        gp_Pnt best;
        double bestDistance = std::numeric_limits<double>::max();
        const auto& edges = myVertexEdges.FindFromKey(vertex);
        for (const auto& e : edges) {
            const TopoDS_Edge& edge = TopoDS::Edge(e);
            if (BRep_Tool::Degenerated(edge) || !myEdgeFaces.Contains(edge))
                continue;
            const auto& onEdge = myEdgeFaces.FindFromKey(edge);
            auto curve = curveAfter(edge);
            if (curve.IsNull())
                continue;
            for (const auto& f : faces) {
                bool own = false;
                for (const auto& g : onEdge)
                    own = own || g.IsSame(f);
                if (own)
                    continue;
                GeomAPI_IntCS cut(curve, surfaceAfter(TopoDS::Face(f)));
                if (!cut.IsDone())
                    continue;
                for (int k = 1; k <= cut.NbPoints(); ++k) {
                    const double d = cut.Point(k).Distance(reference);
                    if (d < bestDistance) {
                        bestDistance = d;
                        best = cut.Point(k);
                        placed = true;
                    }
                }
            }
        }
        if (!placed) {
            // No third face (a vertex on a seam): the nearest point of a moved edge's curve.
            for (const auto& e : edges) {
                const TopoDS_Edge& edge = TopoDS::Edge(e);
                if (BRep_Tool::Degenerated(edge) || !myCurves.IsBound(edge))
                    continue;
                double param = 0.0;
                double distance = 0.0;
                if (nearestOn(myCurves.Find(edge), reference, param, distance) &&
                    distance < bestDistance) {
                    bestDistance = distance;
                    best = myCurves.Find(edge)->Value(param);
                    placed = true;
                }
            }
        }
        if (!placed)
            throw std::runtime_error("tweak: a corner could not be found again");
        myPoints.Bind(vertex, best);
    }

    // Faces and edges that do not move but have a corner that does: they reach to the new corner
    // on their own, unbounded, geometry, and get their pcurves and parameters worked out again.
    for (int i = 1; i <= myVertexFaces.Extent(); ++i) {
        const TopoDS_Vertex& vertex = TopoDS::Vertex(myVertexFaces.FindKey(i));
        if (!myPoints.IsBound(vertex))
            continue;
        for (const auto& f : myVertexFaces.FindFromIndex(i))
            if (!mySurfaces.IsBound(f))
                mySurfaces.Bind(f, unbounded(BRep_Tool::Surface(TopoDS::Face(f))));
        for (const auto& e : myVertexEdges.FindFromKey(vertex)) {
            const TopoDS_Edge& edge = TopoDS::Edge(e);
            if (BRep_Tool::Degenerated(edge) || myCurves.IsBound(edge))
                continue;
            double first = 0.0;
            double last = 0.0;
            auto old = unbounded(BRep_Tool::Curve(edge, first, last));
            if (old.IsNull())
                continue;
            // A spline edge ends where it ends; between two faces it is found again whole.
            const bool bounded =
                old->IsKind(STANDARD_TYPE(Geom_BoundedCurve)) && !old->IsPeriodic();
            if (bounded && myEdgeFaces.Contains(edge) &&
                myEdgeFaces.FindFromKey(edge).Extent() == 2) {
                const auto& faces = myEdgeFaces.FindFromKey(edge);
                gp_Pnt p0;
                gp_Vec tangent;
                old->D1(0.5 * (first + last), p0, tangent);
                myCurves.Bind(edge, meetingOf(TopoDS::Face(faces.First()),
                                              TopoDS::Face(faces.Last()), p0, tangent));
            } else {
                myCurves.Bind(edge, old);
            }
        }
    }
}

int BezierTweakModification::moveOf(const TopoDS_Shape& face) const {
    const int* found = myFaceMove.Seek(face);
    return found == nullptr ? -1 : *found;
}

occ::handle<Geom_Surface> BezierTweakModification::surfaceAfter(const TopoDS_Face& face) const {
    if (mySurfaces.IsBound(face))
        return mySurfaces.Find(face);
    return unbounded(BRep_Tool::Surface(face));
}

occ::handle<Geom_Curve> BezierTweakModification::curveAfter(const TopoDS_Edge& edge) const {
    if (myCurves.IsBound(edge))
        return myCurves.Find(edge);
    double first = 0.0;
    double last = 0.0;
    return unbounded(BRep_Tool::Curve(edge, first, last));
}

occ::handle<Geom_Curve> BezierTweakModification::meetingOf(const TopoDS_Face& a,
                                                           const TopoDS_Face& b,
                                                           const gp_Pnt& where,
                                                           const gp_Vec& tangent) const {
    GeomAPI_IntSS meet(surfaceAfter(a), surfaceAfter(b), Precision::Confusion());
    if (!meet.IsDone() || meet.NbLines() == 0)
        throw std::runtime_error("tweak: two faces would no longer meet");
    // Where the edge was, carried by each face's motion and not at all: one of these may sit
    // where every point of a candidate is equally near (the axis of a circle) and say nothing.
    std::vector<gp_Trsf> motions{gp_Trsf()};
    for (const auto& face : {a, b})
        if (moveOf(face) >= 0)
            motions.push_back(myMoves[static_cast<size_t>(moveOf(face))].motion);
    // Of the curves the two surfaces meet along, the one nearest where the edge was taken.
    occ::handle<Geom_Curve> best;
    double bestDistance = std::numeric_limits<double>::max();
    double bestParam = 0.0;
    gp_Vec bestTangent = tangent;
    for (int k = 1; k <= meet.NbLines(); ++k) {
        // The intersection hands back a stretch of the curve; the edge may need more of it.
        const auto line = unbounded(meet.Line(k));
        for (const auto& motion : motions) {
            double param = 0.0;
            double distance = 0.0;
            if (nearestOn(line, where.Transformed(motion), param, distance) &&
                distance < bestDistance) {
                bestDistance = distance;
                bestParam = param;
                bestTangent = tangent.Transformed(motion);
                best = line;
            }
        }
    }
    if (best.IsNull())
        throw std::runtime_error("tweak: an edge could not be found again");
    // Runs the way the old edge ran, or its vertices would come out in the wrong order.
    gp_Pnt p1;
    gp_Vec along;
    best->D1(bestParam, p1, along);
    return along.Dot(bestTangent) < 0.0 ? best->Reversed() : best;
}

bool BezierTweakModification::carried(const TopoDS_Edge& edge) const {
    if (!myEdgeFaces.Contains(edge))
        return false;
    int move = -2;
    for (const auto& f : myEdgeFaces.FindFromKey(edge)) {
        const int m = moveOf(f);
        if (m < 0 || (move != -2 && m != move))
            return false;
        move = m;
    }
    if (move < 0 || std::abs(myMoves[static_cast<size_t>(move)].offset) > Precision::Confusion())
        return false;
    TopoDS_Vertex v1;
    TopoDS_Vertex v2;
    TopExp::Vertices(edge, v1, v2);
    for (const auto& v : {v1, v2}) {
        if (v.IsNull() || !myVertexFaces.Contains(v))
            continue;
        for (const auto& f : myVertexFaces.FindFromKey(v))
            if (moveOf(f) != move)
                return false;
    }
    return true;
}

bool BezierTweakModification::parameterOn(const TopoDS_Vertex& vertex, const TopoDS_Edge& edge,
                                          double& param, double& distance) const {
    const auto curve = curveAfter(edge);
    if (curve.IsNull())
        return false;
    const gp_Pnt point = myPoints.IsBound(vertex) ? myPoints.Find(vertex) : BRep_Tool::Pnt(vertex);
    if (!nearestOn(curve, point, param, distance))
        return false;
    if (!curve->IsPeriodic()) {
        // A closed edge on a curve that closes without being periodic runs its whole length.
        TopoDS_Vertex v1;
        TopoDS_Vertex v2;
        TopExp::Vertices(edge, v1, v2);
        if (!v1.IsNull() && v1.IsSame(v2) && curve->IsClosed())
            param = vertex.Orientation() == TopAbs_REVERSED ? curve->LastParameter()
                                                            : curve->FirstParameter();
        return true;
    }
    const double period = curve->Period();
    const double start = curve->FirstParameter();
    param -= periodsBetween(start, param, period);
    if (param < start - Precision::PConfusion())
        param += period;
    if (vertex.Orientation() != TopAbs_REVERSED)
        return true;
    // The end of a periodic edge lies after its start, by up to a whole period.
    TopoDS_Vertex v1;
    TopoDS_Vertex v2;
    TopExp::Vertices(edge, v1, v2);
    double from = 0.0;
    double gap = 0.0;
    if (v1.IsNull() || !parameterOn(v1, edge, from, gap))
        return true;
    while (param <= from + Precision::PConfusion())
        param += period;
    while (param > from + period + Precision::PConfusion())
        param -= period;
    return true;
}

bool BezierTweakModification::NewSurface(const TopoDS_Face& F, occ::handle<Geom_Surface>& S,
                                         TopLoc_Location& L, double& Tol, bool& RevWires,
                                         bool& RevFace) {
    RevWires = false;
    RevFace = false;
    if (!mySurfaces.IsBound(F))
        return false;
    S = mySurfaces.Find(F);
    L = TopLoc_Location();
    Tol = BRep_Tool::Tolerance(F);
    return true;
}

bool BezierTweakModification::NewCurve(const TopoDS_Edge& E, occ::handle<Geom_Curve>& C,
                                       TopLoc_Location& L, double& Tol) {
    if (!myCurves.IsBound(E))
        return false;
    C = myCurves.Find(E);
    L = TopLoc_Location();
    Tol = BRep_Tool::Tolerance(E);
    return true;
}

bool BezierTweakModification::NewPoint(const TopoDS_Vertex& V, gp_Pnt& P, double& Tol) {
    if (!myPoints.IsBound(V))
        return false;
    P = myPoints.Find(V);
    Tol = BRep_Tool::Tolerance(V);
    return true;
}

bool BezierTweakModification::NewCurve2d(const TopoDS_Edge& E, const TopoDS_Face& F,
                                         const TopoDS_Edge&, const TopoDS_Face&,
                                         occ::handle<Geom2d_Curve>& C, double& Tol) {
    if (!myCurves.IsBound(E) && !mySurfaces.IsBound(F))
        return false;
    // Carried along with its face, an edge keeps its place on the face's own parameters.
    if (moveOf(F) >= 0 && carried(E))
        return false;
    const auto curve = curveAfter(E);
    if (curve.IsNull())
        return false;
    // The range the edge will have: the one NewParameter gives its vertices.
    TopoDS_Vertex v1;
    TopoDS_Vertex v2;
    TopExp::Vertices(E, v1, v2);
    double first = curve->FirstParameter();
    double last = curve->LastParameter();
    double gap = 0.0;
    if (!v1.IsNull() && !v2.IsNull()) {
        if (!parameterOn(v1, E, first, gap) || !parameterOn(v2, E, last, gap))
            return false;
    }
    if (Precision::IsInfinite(first) || Precision::IsInfinite(last) || last <= first)
        throw std::runtime_error("tweak: an edge would run backwards");
    const auto surface = surfaceAfter(F);
    Tol = BRep_Tool::Tolerance(E);
    C = unbounded(GeomProjLib::Curve2d(curve, first, last, surface, Tol));
    if (C.IsNull())
        throw std::runtime_error("tweak: an edge could not be laid on its face");
    // On a closed surface the projection may land a period away from where the edge was, and a
    // seam's two pcurves must keep their own sides: match the old pcurve at the same vertex.
    double oldFirst = 0.0;
    double oldLast = 0.0;
    const auto before = BRep_Tool::CurveOnSurface(E, F, oldFirst, oldLast);
    if (!before.IsNull() && (surface->IsUPeriodic() || surface->IsVPeriodic())) {
        const gp_Pnt2d was = before->Value(oldFirst);
        const gp_Pnt2d now = C->Value(first);
        const double du =
            surface->IsUPeriodic() ? periodsBetween(now.X(), was.X(), surface->UPeriod()) : 0.0;
        const double dv =
            surface->IsVPeriodic() ? periodsBetween(now.Y(), was.Y(), surface->VPeriod()) : 0.0;
        if (du != 0.0 || dv != 0.0)
            C->Translate(gp_Vec2d(du, dv));
    }
    return true;
}

bool BezierTweakModification::NewParameter(const TopoDS_Vertex& V, const TopoDS_Edge& E, double& P,
                                           double& Tol) {
    if (!myPoints.IsBound(V) && !myCurves.IsBound(E))
        return false;
    if (carried(E))
        return false;
    // The modifier walks a reversed edge with its vertices' orientations composed; which end a
    // vertex is follows from its orientation in the edge itself.
    TopoDS_Vertex end = V;
    if (E.Orientation() == TopAbs_REVERSED)
        end.Reverse();
    double param = 0.0;
    double distance = 0.0;
    if (!parameterOn(end, E, param, distance))
        return false;
    P = param;
    Tol = std::max(BRep_Tool::Tolerance(V), distance);
    return true;
}

GeomAbs_Shape BezierTweakModification::Continuity(const TopoDS_Edge& E, const TopoDS_Face& F1,
                                                  const TopoDS_Face& F2, const TopoDS_Edge&,
                                                  const TopoDS_Face&, const TopoDS_Face&) {
    return BRep_Tool::Continuity(E, F1, F2);
}

std::string BezierTweakProblem(const TopoDS_Shape& shape) {
    BRepCheck_Analyzer analyzer(shape, true);
    if (analyzer.IsValid())
        return {};
    const char* names[] = {"vertex", "edge", "wire", "face", "shell", "solid"};
    const TopAbs_ShapeEnum types[] = {TopAbs_VERTEX, TopAbs_EDGE,  TopAbs_WIRE,
                                      TopAbs_FACE,   TopAbs_SHELL, TopAbs_SOLID};
    for (int t = 0; t < 6; ++t) {
        int index = 0;
        for (TopExp_Explorer it(shape, types[t]); it.More(); it.Next()) {
            ++index;
            const auto& result = analyzer.Result(it.Current());
            if (result.IsNull())
                continue;
            std::ostringstream out;
            for (const auto& status : result->Status())
                if (status != BRepCheck_NoError)
                    BRepCheck::Print(status, out);
            for (result->InitContextIterator(); result->MoreShapeInContext();
                 result->NextShapeInContext())
                for (const auto& status : result->StatusOnShape())
                    if (status != BRepCheck_NoError)
                        BRepCheck::Print(status, out);
            const std::string said = out.str();
            if (!said.empty())
                return std::string(names[t]) + " " + std::to_string(index) + ": " + said;
        }
    }
    return "the shape is not valid";
}

BezierTweak::BezierTweak(const TopoDS_Shape& shape, const std::vector<BezierFaceMove>& moves)
    : BRepBuilderAPI_ModifyShape(
          occ::handle<BRepTools_Modification>(new BezierTweakModification(shape, moves))) {
    try {
        DoModif(shape);
    } catch (const Standard_Failure& failure) {
        // An OCCT failure often comes without a message; its type is what says where it was.
        throw std::runtime_error(std::string("tweak: ") + failure.ExceptionType() + " " +
                                 failure.what());
    }
    if (!IsDone())
        return;
    // New edges were laid on their faces by projection; settle the tolerances that leaves.
    try {
        BRepLib::SameParameter(myShape, Precision::Confusion(), false);
        BRepLib::UpdateTolerances(myShape, true);
    } catch (const Standard_Failure& failure) {
        throw std::runtime_error(std::string("tweak (tolerances): ") + failure.ExceptionType() +
                                 " " + failure.what());
    }
}
