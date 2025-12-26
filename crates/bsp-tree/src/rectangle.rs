//! Rectangle (quad) representation for BSP trees.

use nalgebra::{Point3, Vector3};

use crate::{Classification, Plane3D, PlaneSide};

/// A rectangle (quad) in 3D space, defined by a corner and two edge vectors.
///
/// The four vertices are:
/// - `origin`
/// - `origin + u`
/// - `origin + u + v`
/// - `origin + v`
#[derive(Debug, Clone, PartialEq)]
pub struct Rectangle {
    origin: Point3<f32>,
    u: Vector3<f32>,
    v: Vector3<f32>,
}

impl Rectangle {
    /// Creates a new rectangle from an origin corner and two edge vectors.
    ///
    /// The vertices will be: origin, origin+u, origin+u+v, origin+v (counter-clockwise).
    pub fn new(origin: Point3<f32>, u: Vector3<f32>, v: Vector3<f32>) -> Self {
        Self { origin, u, v }
    }

    /// Creates a rectangle from four corner points.
    ///
    /// The winding order should be: a -> b -> c -> d (counter-clockwise).
    ///
    /// Internally computes u = b - a and v = d - a.
    ///
    /// # Panics (debug builds only)
    /// Panics if the points are not coplanar.
    pub fn from_corners(
        a: Point3<f32>,
        b: Point3<f32>,
        c: Point3<f32>,
        d: Point3<f32>,
    ) -> Self {
        debug_assert!(
            {
                let plane = Plane3D::from_three_points(a, b, d);
                plane.classify_point(c) == PlaneSide::OnPlane
            },
            "Rectangle corners must be coplanar"
        );
        let u = b - a;
        let v = d - a;
        Self { origin: a, u, v }
    }

    /// Returns the origin corner of the rectangle.
    #[inline]
    pub fn origin(&self) -> Point3<f32> {
        self.origin
    }

    /// Returns the first edge vector.
    #[inline]
    pub fn u(&self) -> Vector3<f32> {
        self.u
    }

    /// Returns the second edge vector.
    #[inline]
    pub fn v(&self) -> Vector3<f32> {
        self.v
    }

    /// Returns the four vertices of the rectangle.
    ///
    /// Order: origin, origin+u, origin+u+v, origin+v (counter-clockwise).
    pub fn vertices(&self) -> [Point3<f32>; 4] {
        [
            self.origin,
            self.origin + self.u,
            self.origin + self.u + self.v,
            self.origin + self.v,
        ]
    }

    /// Computes the (unnormalized) normal vector of the rectangle.
    ///
    /// The direction follows the right-hand rule: u × v.
    pub fn normal(&self) -> Vector3<f32> {
        self.u.cross(&self.v)
    }

    /// Computes the unit normal vector of the rectangle.
    ///
    /// Returns `None` if the rectangle is degenerate (zero area).
    pub fn unit_normal(&self) -> Option<Vector3<f32>> {
        let n = self.normal();
        let len = n.norm();
        if len > f32::EPSILON {
            Some(n / len)
        } else {
            None
        }
    }

    /// Returns the plane that this rectangle lies on.
    ///
    /// # Panics
    /// Panics if the rectangle is degenerate (u and v are parallel).
    pub fn plane(&self) -> Plane3D {
        Plane3D::from_point_and_normal(self.origin, self.normal())
    }

    /// Computes the centroid (center) of the rectangle.
    pub fn centroid(&self) -> Point3<f32> {
        self.origin + (self.u + self.v) * 0.5
    }

    /// Computes the area of the rectangle.
    pub fn area(&self) -> f32 {
        self.normal().norm()
    }

    /// Classifies this rectangle relative to a plane.
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

        for vertex in self.vertices() {
            match plane.classify_point(vertex) {
                PlaneSide::Front => front += 1,
                PlaneSide::Back => back += 1,
                PlaneSide::OnPlane => on_plane += 1,
            }
        }

        if on_plane == 4 {
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

impl From<Rectangle> for Plane3D {
    fn from(rectangle: Rectangle) -> Self {
        rectangle.plane()
    }
}

impl From<&Rectangle> for Plane3D {
    fn from(rectangle: &Rectangle) -> Self {
        rectangle.plane()
    }
}
