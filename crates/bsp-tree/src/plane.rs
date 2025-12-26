//! Plane representation and operations for BSP trees.

use nalgebra::{Point3, Vector3};

/// Default epsilon for plane classification.
/// Points within this distance of the plane are considered "on" the plane.
pub const PLANE_EPSILON: f32 = 1e-5;

/// Which side of a plane a point lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneSide {
    /// Point is in front of the plane (positive side of normal)
    Front,
    /// Point is behind the plane (negative side of normal)
    Back,
    /// Point lies on the plane (within epsilon tolerance)
    OnPlane,
}

/// Classification of geometry (polygon, triangle, rectangle) relative to a plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Classification {
    /// All vertices are in front of the plane
    Front,
    /// All vertices are behind the plane
    Back,
    /// All vertices are on the plane (coplanar)
    Coplanar,
    /// Vertices are on both sides (spans the plane)
    Spanning,
}

/// A plane in 3D space, represented as `normal · point = offset`.
#[derive(Debug, Clone, PartialEq)]
pub struct Plane3D {
    normal: Vector3<f32>,
    offset: f32,
}

impl Plane3D {
    /// Creates a new plane from a normal vector and offset.
    /// The normal will be normalized automatically.
    ///
    /// # Panics
    /// Panics if the normal vector has zero length.
    pub fn new(normal: Vector3<f32>, offset: f32) -> Self {
        let norm = normal.norm();
        assert!(norm > f32::EPSILON, "Plane normal cannot be zero");
        Self {
            normal: normal / norm,
            offset: offset / norm,
        }
    }

    /// Creates a plane from a point on the plane and a normal vector.
    /// The normal will be normalized automatically.
    ///
    /// # Panics
    /// Panics if the normal vector has zero length.
    pub fn from_point_and_normal(point: Point3<f32>, normal: Vector3<f32>) -> Self {
        let norm = normal.norm();
        assert!(norm > f32::EPSILON, "Plane normal cannot be zero");
        let unit_normal = normal / norm;
        let offset = unit_normal.dot(&point.coords);
        Self {
            normal: unit_normal,
            offset,
        }
    }

    /// Creates a plane from three non-collinear points.
    /// The normal direction follows the right-hand rule: (b - a) × (c - a).
    ///
    /// # Panics
    /// Panics if the points are collinear (or nearly so).
    pub fn from_three_points(a: Point3<f32>, b: Point3<f32>, c: Point3<f32>) -> Self {
        let ab = b - a;
        let ac = c - a;
        let normal = ab.cross(&ac);
        Self::from_point_and_normal(a, normal)
    }

    /// Returns the unit normal vector of the plane.
    #[inline]
    pub fn normal(&self) -> Vector3<f32> {
        self.normal
    }

    /// Returns the signed distance from the origin to the plane along the normal.
    #[inline]
    pub fn offset(&self) -> f32 {
        self.offset
    }

    /// Computes the signed distance from a point to the plane.
    /// - Positive: point is in front (same side as normal)
    /// - Negative: point is behind (opposite side from normal)
    /// - Zero: point is on the plane
    #[inline]
    pub fn signed_distance(&self, point: Point3<f32>) -> f32 {
        self.normal.dot(&point.coords) - self.offset
    }

    /// Classifies which side of the plane a point lies on.
    /// Uses the default `PLANE_EPSILON` tolerance.
    #[inline]
    pub fn classify_point(&self, point: Point3<f32>) -> PlaneSide {
        self.classify_point_with_epsilon(point, PLANE_EPSILON)
    }

    /// Classifies which side of the plane a point lies on, with a custom epsilon.
    pub fn classify_point_with_epsilon(&self, point: Point3<f32>, epsilon: f32) -> PlaneSide {
        let dist = self.signed_distance(point);
        if dist > epsilon {
            PlaneSide::Front
        } else if dist < -epsilon {
            PlaneSide::Back
        } else {
            PlaneSide::OnPlane
        }
    }

    /// Returns a new plane with the normal flipped (facing the opposite direction).
    #[inline]
    pub fn flipped(&self) -> Self {
        Self {
            normal: -self.normal,
            offset: -self.offset,
        }
    }

    /// Projects a point onto the plane (finds the closest point on the plane).
    #[inline]
    pub fn project_point(&self, point: Point3<f32>) -> Point3<f32> {
        point - self.normal * self.signed_distance(point)
    }

    /// Computes the intersection of a line segment with the plane.
    ///
    /// Returns `Some((t, point))` where:
    /// - `t` is the interpolation parameter (0.0 = start, 1.0 = end)
    /// - `point` is the intersection point
    ///
    /// Returns `None` if the segment is parallel to the plane or doesn't intersect.
    pub fn intersect_segment(
        &self,
        start: Point3<f32>,
        end: Point3<f32>,
    ) -> Option<(f32, Point3<f32>)> {
        let direction = end - start;
        let denom = self.normal.dot(&direction);

        // Segment is parallel to plane
        if denom.abs() < f32::EPSILON {
            return None;
        }

        let t = (self.offset - self.normal.dot(&start.coords)) / denom;

        // Intersection is outside the segment
        if t < 0.0 || t > 1.0 {
            return None;
        }

        let point = start + direction * t;
        Some((t, point))
    }
}
