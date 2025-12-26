//! Generic polygon representation for BSP trees.

use nalgebra::{Point3, Vector3};

use crate::{Classification, Plane3D, PlaneSide, Rectangle, Triangle};

/// A convex polygon in 3D space, defined by an ordered list of vertices.
///
/// Vertices should be coplanar and in counter-clockwise winding order
/// when viewed from the front (the direction the normal points).
#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    vertices: Vec<Point3<f32>>,
}

impl Polygon {
    /// Creates a new polygon from a list of vertices.
    ///
    /// # Panics (debug builds only)
    /// - Panics if fewer than 3 vertices are provided.
    /// - Panics if vertices are not coplanar.
    pub fn new(vertices: Vec<Point3<f32>>) -> Self {
        debug_assert!(
            vertices.len() >= 3,
            "Polygon must have at least 3 vertices"
        );
        debug_assert!(
            Self::are_coplanar(&vertices),
            "Polygon vertices must be coplanar"
        );
        Self { vertices }
    }

    /// Checks if all vertices lie on the same plane.
    fn are_coplanar(vertices: &[Point3<f32>]) -> bool {
        if vertices.len() <= 3 {
            return true;
        }

        let plane = Plane3D::from_three_points(vertices[0], vertices[1], vertices[2]);
        vertices[3..]
            .iter()
            .all(|v| plane.classify_point(*v) == PlaneSide::OnPlane)
    }

    /// Returns the vertices of the polygon.
    #[inline]
    pub fn vertices(&self) -> &[Point3<f32>] {
        &self.vertices
    }

    /// Returns the number of vertices.
    #[inline]
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Returns true if the polygon has no vertices (always false for valid polygons).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    /// Computes the (unnormalized) normal vector of the polygon.
    ///
    /// Uses the first three vertices to compute the normal via cross product.
    /// The direction follows the right-hand rule based on vertex winding.
    pub fn normal(&self) -> Vector3<f32> {
        let a = &self.vertices[0];
        let b = &self.vertices[1];
        let c = &self.vertices[2];
        let ab = b - a;
        let ac = c - a;
        ab.cross(&ac)
    }

    /// Computes the unit normal vector of the polygon.
    ///
    /// Returns `None` if the first three vertices are collinear.
    pub fn unit_normal(&self) -> Option<Vector3<f32>> {
        let n = self.normal();
        let len = n.norm();
        if len > f32::EPSILON {
            Some(n / len)
        } else {
            None
        }
    }

    /// Returns the plane that this polygon lies on.
    ///
    /// # Panics
    /// Panics if the first three vertices are collinear.
    pub fn plane(&self) -> Plane3D {
        Plane3D::from_three_points(self.vertices[0], self.vertices[1], self.vertices[2])
    }

    /// Computes the centroid (center of mass) of the polygon.
    pub fn centroid(&self) -> Point3<f32> {
        let sum: Vector3<f32> = self.vertices.iter().map(|p| p.coords).sum();
        Point3::from(sum / self.vertices.len() as f32)
    }

    /// Classifies this polygon relative to a plane.
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

        if on_plane == self.vertices.len() {
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

impl From<Triangle> for Polygon {
    fn from(triangle: Triangle) -> Self {
        Self {
            vertices: triangle.vertices().to_vec(),
        }
    }
}

impl From<&Triangle> for Polygon {
    fn from(triangle: &Triangle) -> Self {
        Self {
            vertices: triangle.vertices().to_vec(),
        }
    }
}

impl From<Rectangle> for Polygon {
    fn from(rectangle: Rectangle) -> Self {
        Self {
            vertices: rectangle.vertices().to_vec(),
        }
    }
}

impl From<&Rectangle> for Polygon {
    fn from(rectangle: &Rectangle) -> Self {
        Self {
            vertices: rectangle.vertices().to_vec(),
        }
    }
}

impl From<Polygon> for Plane3D {
    fn from(polygon: Polygon) -> Self {
        polygon.plane()
    }
}

impl From<&Polygon> for Plane3D {
    fn from(polygon: &Polygon) -> Self {
        polygon.plane()
    }
}
