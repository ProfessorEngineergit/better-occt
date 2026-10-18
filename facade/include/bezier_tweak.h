// Moving faces of a solid and letting the rest of it follow: the "tweak" of direct modellers.
//
// Each moved face gets a new surface (its own, carried by a rigid motion and then offset along its
// outward normal). Every edge that touches a moved face is recomputed as the intersection of its
// two faces' new surfaces, every vertex as the meeting of its new edges with the faces around it.
// Faces that do not move keep their surfaces, extended as far as the new edges need. The topology
// stays what it was; a move that would change it is refused.
//
// Built on BRepTools_Modification, the interface OCCT's own draft angle (Draft_Modification) uses,
// and applied by BRepTools_Modifier, so the result carries Modified() history like any other
// operation.

#pragma once

#include <BRepBuilderAPI_ModifyShape.hxx>
#include <BRepTools_Modification.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <NCollection_DataMap.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <string>
#include <vector>

/// One face's move: a rigid motion, then an offset along the face's outward normal. Or, `ruled`,
/// the face is replaced by the surface ruled between two segments: `from` (where its far side
/// stays) and `to` (where the edge that was turned now runs). That is how a flat face follows an
/// edge turned so that the face can no longer stay flat: it twists, as in other direct modellers.
struct BezierFaceMove {
    TopoDS_Shape face;
    gp_Trsf motion;
    double offset = 0.0;
    bool ruled = false;
    gp_Pnt from[2];
    gp_Pnt to[2];
};

class BezierTweakModification : public BRepTools_Modification {
  public:
    /// Works out every new surface, curve and point up front; throws with a sentence if a move has
    /// no solution (parallel faces that no longer meet, a corner that has gone).
    BezierTweakModification(const TopoDS_Shape& shape, const std::vector<BezierFaceMove>& moves);

    bool NewSurface(const TopoDS_Face& F, occ::handle<Geom_Surface>& S, TopLoc_Location& L,
                    double& Tol, bool& RevWires, bool& RevFace) override;
    bool NewCurve(const TopoDS_Edge& E, occ::handle<Geom_Curve>& C, TopLoc_Location& L,
                  double& Tol) override;
    bool NewPoint(const TopoDS_Vertex& V, gp_Pnt& P, double& Tol) override;
    bool NewCurve2d(const TopoDS_Edge& E, const TopoDS_Face& F, const TopoDS_Edge& NewE,
                    const TopoDS_Face& NewF, occ::handle<Geom2d_Curve>& C, double& Tol) override;
    bool NewParameter(const TopoDS_Vertex& V, const TopoDS_Edge& E, double& P,
                      double& Tol) override;
    GeomAbs_Shape Continuity(const TopoDS_Edge& E, const TopoDS_Face& F1, const TopoDS_Face& F2,
                             const TopoDS_Edge& NewE, const TopoDS_Face& NewF1,
                             const TopoDS_Face& NewF2) override;

  private:
    using ShapeMap = NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>,
                                                TopTools_ShapeMapHasher>;

    /// Works out the new geometry; OCCT failures in it are turned into sentences by the caller.
    void prepare(const TopoDS_Shape& shape);
    /// The surface a face stands on after the move: unbounded, in world coordinates.
    occ::handle<Geom_Surface> surfaceAfter(const TopoDS_Face& face) const;
    /// The curve an edge lies on after the move, unbounded where the old one was trimmed.
    occ::handle<Geom_Curve> curveAfter(const TopoDS_Edge& edge) const;
    /// Where two faces meet after the move: the branch nearest the old edge's point `where`
    /// (as it was, or carried by either face's motion), running the way it ran; throws when
    /// they no longer meet.
    occ::handle<Geom_Curve> meetingOf(const TopoDS_Face& a, const TopoDS_Face& b,
                                      const gp_Pnt& where, const gp_Vec& tangent) const;
    /// Where a vertex sits on its edge's new curve. On a periodic curve the edge's end lies after
    /// its start, a period on for a closed edge, so edge and pcurves agree on one range.
    bool parameterOn(const TopoDS_Vertex& vertex, const TopoDS_Edge& edge, double& param,
                     double& distance) const;
    /// True when an edge and both its vertices simply ride along with one rigid motion, so
    /// everything it stored still holds.
    bool carried(const TopoDS_Edge& edge) const;
    /// The move a face takes part in, or -1.
    int moveOf(const TopoDS_Shape& face) const;

    std::vector<BezierFaceMove> myMoves;
    NCollection_DataMap<TopoDS_Shape, int, TopTools_ShapeMapHasher> myFaceMove;
    NCollection_DataMap<TopoDS_Shape, occ::handle<Geom_Surface>, TopTools_ShapeMapHasher>
        mySurfaces;
    NCollection_DataMap<TopoDS_Shape, occ::handle<Geom_Curve>, TopTools_ShapeMapHasher> myCurves;
    NCollection_DataMap<TopoDS_Shape, gp_Pnt, TopTools_ShapeMapHasher> myPoints;
    ShapeMap myEdgeFaces;
    ShapeMap myVertexFaces;
    ShapeMap myVertexEdges;
};

/// Why a shape fails the full check, in BRepCheck's words, or empty when it passes.
std::string BezierTweakProblem(const TopoDS_Shape& shape);

/// The modification applied to a shape, with the Modified() history a facade method reports.
class BezierTweak : public BRepBuilderAPI_ModifyShape {
  public:
    BezierTweak(const TopoDS_Shape& shape, const std::vector<BezierFaceMove>& moves);
};
