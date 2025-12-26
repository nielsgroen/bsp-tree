//! Triangle representation for BSP trees.

use nalgebra::{Point3, Vector3};

use crate::{Classification, Plane3D, PlaneSide};

/// A triangle in 3D space, defined by three vertices.
#[derive(Debug, Clone, PartialEq)]
pub struct Triangle {
    vertices: [Point3<f32>; 3],
}

impl Triangle {
    /// Creates a new triangle from three points.
    ///
    /// The winding order determines the normal direction via the right-hand rule:
    /// normal = (b - a) × (c - a)
    pub fn new(a: Point3<f32>, b: Point3<f32>, c: Point3<f32>) -> Self {
        Self {
            vertices: [a, b, c],
        }
    }

    /// Returns the three vertices of the triangle.
    #[inline]
    pub fn vertices(&self) -> &[Point3<f32>; 3] {
        &self.vertices
    }

    /// Computes the (unnormalized) normal vector of the triangle.
    ///
    /// The direction follows the right-hand rule based on vertex winding.
    pub fn normal(&self) -> Vector3<f32> {
        let [a, b, c] = &self.vertices;
        let ab = b - a;
        let ac = c - a;
        ab.cross(&ac)
    }

    /// Computes the unit normal vector of the triangle.
    ///
    /// Returns `None` if the triangle is degenerate (zero area).
    pub fn unit_normal(&self) -> Option<Vector3<f32>> {
        let n = self.normal();
        let len = n.norm();
        if len > f32::EPSILON {
            Some(n / len)
        } else {
            None
        }
    }

    /// Returns the plane that this triangle lies on.
    ///
    /// # Panics
    /// Panics if the triangle is degenerate (vertices are collinear).
    pub fn plane(&self) -> Plane3D {
        let [a, b, c] = &self.vertices;
        Plane3D::from_three_points(*a, *b, *c)
    }

    /// Computes the centroid (center of mass) of the triangle.
    pub fn centroid(&self) -> Point3<f32> {
        let [a, b, c] = &self.vertices;
        Point3::from((a.coords + b.coords + c.coords) / 3.0)
    }

    /// Classifies this triangle relative to a plane.
    ///
    /// Returns:
    /// - `Front` if all vertices are in front of the plane
    /// - `Back` if all vertices are behind the plane
    /// - `Coplanar` if all vertices lie on the plane
    /// - `Spanning` if vertices are on both sides
    pub fn classify(&self, plane: &Plane3D) -> Classification {
        let mut front = 0;
        let mut back = 0;
        let mut on_plane = 0;

        for vertex in &self.vertices {
            match plane.classify_point(*vertex) {
                PlaneSide::Front => front += 1,
                PlaneSide::Back => back += 1,
                PlaneSide::OnPlane => on_plane += 1,
            }
        }

        if on_plane == 3 {
            Classification::Coplanar
        } else if back == 0 {
            Classification::Front
        } else if front == 0 {
            Classification::Back
        } else {
            Classification::Spanning
        }
    }
}

impl From<Triangle> for Plane3D {
    fn from(triangle: Triangle) -> Self {
        triangle.plane()
    }
}

impl From<&Triangle> for Plane3D {
    fn from(triangle: &Triangle) -> Self {
        triangle.plane()
    }
}
