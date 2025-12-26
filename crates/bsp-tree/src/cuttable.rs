//! Polygon cutting/splitting operations for BSP trees.

use crate::{Classification, Plane3D, PlaneSide, Polygon, Rectangle, Triangle};

/// Trait for geometry that can be cut by a plane.
pub trait Cuttable {
    /// Cuts the geometry by a plane.
    ///
    /// Returns `(front, back)` where:
    /// - `front`: `Some(polygon)` containing the part on the front side of the plane
    /// - `back`: `Some(polygon)` containing the part on the back side of the plane
    ///
    /// # Return values by classification
    ///
    /// - **Front**: `(Some(self), None)` - entire geometry is in front
    /// - **Back**: `(None, Some(self))` - entire geometry is behind
    /// - **Coplanar**: `(Some(self), None)` - treated as front
    /// - **Spanning**: `(Some(front_part), Some(back_part))` - split into two pieces
    fn cut(&self, plane: &Plane3D) -> (Option<Polygon>, Option<Polygon>);
}

impl Cuttable for Polygon {
    fn cut(&self, plane: &Plane3D) -> (Option<Polygon>, Option<Polygon>) {
        match self.classify(plane) {
            Classification::Front | Classification::Coplanar => {
                (Some(self.clone()), None)
            }
            Classification::Back => {
                (None, Some(self.clone()))
            }
            Classification::Spanning => {
                split_polygon(self, plane)
            }
        }
    }
}

/// Splits a spanning polygon into front and back parts.
///
/// Uses a variant of the Sutherland-Hodgman algorithm:
/// walks the polygon edges and builds two vertex lists,
/// adding intersection points when edges cross the plane.
fn split_polygon(polygon: &Polygon, plane: &Plane3D) -> (Option<Polygon>, Option<Polygon>) {
    let vertices = polygon.vertices();
    let n = vertices.len();

    let mut front_verts = Vec::with_capacity(n + 1);
    let mut back_verts = Vec::with_capacity(n + 1);

    // Classify all vertices upfront
    let sides: Vec<PlaneSide> = vertices
        .iter()
        .map(|v| plane.classify_point(*v))
        .collect();

    for i in 0..n {
        let current = vertices[i];
        let current_side = sides[i];
        let next_idx = (i + 1) % n;
        let next = vertices[next_idx];
        let next_side = sides[next_idx];

        // Add current vertex to appropriate list(s)
        match current_side {
            PlaneSide::Front => front_verts.push(current),
            PlaneSide::Back => back_verts.push(current),
            PlaneSide::OnPlane => {
                // On-plane vertices go to both sides
                front_verts.push(current);
                back_verts.push(current);
            }
        }

        // Check if edge crosses the plane (excluding on-plane cases)
        let crosses = matches!(
            (current_side, next_side),
            (PlaneSide::Front, PlaneSide::Back) | (PlaneSide::Back, PlaneSide::Front)
        );

        if crosses {
            // Compute intersection point
            if let Some((_, intersection)) = plane.intersect_segment(current, next) {
                front_verts.push(intersection);
                back_verts.push(intersection);
            }
        }
    }

    // Build result polygons (only if they have enough vertices)
    let front = if front_verts.len() >= 3 {
        Some(Polygon::new(front_verts))
    } else {
        None
    };

    let back = if back_verts.len() >= 3 {
        Some(Polygon::new(back_verts))
    } else {
        None
    };

    (front, back)
}

impl Cuttable for Triangle {
    fn cut(&self, plane: &Plane3D) -> (Option<Polygon>, Option<Polygon>) {
        Polygon::from(self).cut(plane)
    }
}

impl Cuttable for Rectangle {
    fn cut(&self, plane: &Plane3D) -> (Option<Polygon>, Option<Polygon>) {
        Polygon::from(self).cut(plane)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    // =========================================================================
    // Helper functions
    // =========================================================================

    /// Creates a horizontal plane at the given height (normal pointing up).
    fn horizontal_plane(height: f32) -> Plane3D {
        Plane3D::from_point_and_normal(
            Point3::new(0.0, height, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        )
    }

    /// Creates a vertical plane at x = offset (normal pointing in +X direction).
    fn vertical_plane_x(offset: f32) -> Plane3D {
        Plane3D::from_point_and_normal(
            Point3::new(offset, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        )
    }

    /// Creates a vertical plane at z = offset (normal pointing in +Z direction).
    fn vertical_plane_z(offset: f32) -> Plane3D {
        Plane3D::from_point_and_normal(
            Point3::new(0.0, 0.0, offset),
            Vector3::new(0.0, 0.0, 1.0),
        )
    }

    /// Checks that all vertices of a polygon are on the expected side of the plane.
    fn assert_all_vertices_on_side(polygon: &Polygon, plane: &Plane3D, expected: PlaneSide) {
        for (i, v) in polygon.vertices().iter().enumerate() {
            let side = plane.classify_point(*v);
            assert!(
                side == expected || side == PlaneSide::OnPlane,
                "Vertex {i} at {v:?} is on {side:?}, expected {expected:?} or OnPlane"
            );
        }
    }

    /// Checks that a polygon has the expected number of vertices.
    fn assert_vertex_count(polygon: &Polygon, expected: usize) {
        assert_eq!(
            polygon.len(),
            expected,
            "Expected {expected} vertices, got {}",
            polygon.len()
        );
    }

    /// Approximate equality for f32 with tolerance.
    fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() < epsilon
    }

    /// Checks that a point lies on the plane (within tolerance).
    fn assert_point_on_plane(point: Point3<f32>, plane: &Plane3D) {
        let dist = plane.signed_distance(point).abs();
        assert!(
            dist < 1e-4,
            "Point {point:?} should be on plane, but distance is {dist}"
        );
    }

    /// Checks if two vertex sequences represent the same polygon (rotation-independent).
    ///
    /// Returns true if `actual` contains the same vertices as `expected` in the same
    /// cyclic order, but possibly starting at a different index.
    fn same_polygon_vertices(actual: &[Point3<f32>], expected: &[Point3<f32>]) -> bool {
        if actual.len() != expected.len() {
            return false;
        }
        let n = actual.len();
        if n == 0 {
            return true;
        }

        // Find starting offset: where does expected[0] appear in actual?
        let Some(offset) = actual.iter().position(|v| v == &expected[0]) else {
            return false;
        };

        // Check if all vertices match with this rotation
        for i in 0..n {
            if actual[(offset + i) % n] != expected[i] {
                return false;
            }
        }
        true
    }

    // =========================================================================
    // Polygon: Classification tests (non-spanning)
    // =========================================================================

    #[test]
    fn polygon_entirely_in_front() {
        // Triangle above the XZ plane at y=0
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(0.0, 1.5, 1.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some(), "Front should contain the polygon");
        assert!(back.is_none(), "Back should be empty");
        assert_eq!(front.unwrap().vertices(), polygon.vertices());
    }

    #[test]
    fn polygon_entirely_behind() {
        // Triangle below the XZ plane at y=0
        let polygon = Polygon::new(vec![
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, -2.0, 0.0),
            Point3::new(0.0, -1.5, 1.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_none(), "Front should be empty");
        assert!(back.is_some(), "Back should contain the polygon");
        assert_eq!(back.unwrap().vertices(), polygon.vertices());
    }

    #[test]
    fn polygon_coplanar_with_plane() {
        // Triangle lying exactly on the XZ plane
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 1.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some(), "Coplanar should be treated as front");
        assert!(back.is_none(), "Back should be empty for coplanar");
        assert_eq!(front.unwrap().vertices(), polygon.vertices());
    }

    // =========================================================================
    // Polygon: Spanning/splitting tests
    // =========================================================================

    #[test]
    fn polygon_split_triangle_one_vertex_front() {
        // Triangle with one vertex above plane, two below
        //     * (0, 2, 0) - front
        //    / \
        //   *---* (±1, -1, 0) - both back
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 2.0, 0.0),  // front
            Point3::new(-1.0, -1.0, 0.0), // back
            Point3::new(1.0, -1.0, 0.0),  // back
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some(), "Should have front part");
        assert!(back.is_some(), "Should have back part");

        let front = front.unwrap();
        let back = back.unwrap();

        // Front part should be a triangle (1 original + 2 intersection points)
        assert_vertex_count(&front, 3);
        // Back part should be a quad (2 original + 2 intersection points)
        assert_vertex_count(&back, 4);

        // Verify all front vertices are in front or on plane
        assert_all_vertices_on_side(&front, &plane, PlaneSide::Front);
        // Verify all back vertices are behind or on plane
        assert_all_vertices_on_side(&back, &plane, PlaneSide::Back);
    }

    #[test]
    fn polygon_split_triangle_two_vertices_front() {
        // Triangle with two vertices above plane, one below
        let polygon = Polygon::new(vec![
            Point3::new(-1.0, 1.0, 0.0), // front
            Point3::new(1.0, 1.0, 0.0),  // front
            Point3::new(0.0, -2.0, 0.0), // back
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some(), "Should have front part");
        assert!(back.is_some(), "Should have back part");

        let front = front.unwrap();
        let back = back.unwrap();

        // Front part should be a quad (2 original + 2 intersection points)
        assert_vertex_count(&front, 4);
        // Back part should be a triangle (1 original + 2 intersection points)
        assert_vertex_count(&back, 3);

        assert_all_vertices_on_side(&front, &plane, PlaneSide::Front);
        assert_all_vertices_on_side(&back, &plane, PlaneSide::Back);
    }

    #[test]
    fn polygon_split_quad_in_half() {
        // Square split horizontally through the middle
        // Two vertices above, two below
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 1.0, 0.0),  // front
            Point3::new(1.0, 1.0, 0.0),  // front
            Point3::new(1.0, -1.0, 0.0), // back
            Point3::new(0.0, -1.0, 0.0), // back
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        let front = front.unwrap();
        let back = back.unwrap();

        // Both halves should be quads
        assert_vertex_count(&front, 4);
        assert_vertex_count(&back, 4);

        assert_all_vertices_on_side(&front, &plane, PlaneSide::Front);
        assert_all_vertices_on_side(&back, &plane, PlaneSide::Back);
    }

    #[test]
    fn polygon_split_pentagon() {
        // Pentagon split by a plane
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 2.0, 0.0),   // front
            Point3::new(2.0, 1.0, 0.0),   // front
            Point3::new(1.5, -1.0, 0.0),  // back
            Point3::new(-1.5, -1.0, 0.0), // back
            Point3::new(-2.0, 1.0, 0.0),  // front
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        let front = front.unwrap();
        let back = back.unwrap();

        // Front: 3 original vertices + 2 intersection points = 5
        assert_vertex_count(&front, 5);
        // Back: 2 original vertices + 2 intersection points = 4
        assert_vertex_count(&back, 4);

        assert_all_vertices_on_side(&front, &plane, PlaneSide::Front);
        assert_all_vertices_on_side(&back, &plane, PlaneSide::Back);
    }

    #[test]
    fn polygon_split_preserves_intersection_points_on_plane() {
        // Verify that intersection points actually lie on the cutting plane
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(2.0, -1.0, 1.0),
            Point3::new(-2.0, -1.0, -1.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        let front = front.unwrap();
        let back = back.unwrap();

        // Find intersection points (those that appear in both polygons)
        for fv in front.vertices() {
            if plane.classify_point(*fv) == PlaneSide::OnPlane {
                assert_point_on_plane(*fv, &plane);
            }
        }
        for bv in back.vertices() {
            if plane.classify_point(*bv) == PlaneSide::OnPlane {
                assert_point_on_plane(*bv, &plane);
            }
        }
    }

    // =========================================================================
    // Polygon: Edge cases with vertices on the plane
    // =========================================================================

    #[test]
    fn polygon_one_vertex_exactly_on_plane() {
        // Triangle with one vertex on the plane, others in front
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.0, 0.0), // on plane
            Point3::new(1.0, 1.0, 0.0), // front
            Point3::new(-1.0, 1.0, 0.0), // front
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        // Should classify as front (no vertices behind)
        assert!(front.is_some());
        assert!(back.is_none());
    }

    #[test]
    fn polygon_one_vertex_on_plane_spanning() {
        // Triangle with one vertex on plane, one front, one back
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.0, 0.0),  // on plane
            Point3::new(1.0, 1.0, 0.0),  // front
            Point3::new(1.0, -1.0, 0.0), // back
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        let front = front.unwrap();
        let back = back.unwrap();

        // The on-plane vertex should appear in both polygons
        let on_plane_point = Point3::new(0.0, 0.0, 0.0);
        assert!(
            front.vertices().contains(&on_plane_point),
            "On-plane vertex should be in front polygon"
        );
        assert!(
            back.vertices().contains(&on_plane_point),
            "On-plane vertex should be in back polygon"
        );
    }

    #[test]
    fn polygon_edge_lies_on_plane() {
        // Square where one edge lies exactly on the cutting plane
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.0, 0.0), // on plane
            Point3::new(1.0, 0.0, 0.0), // on plane
            Point3::new(1.0, 1.0, 0.0), // front
            Point3::new(0.0, 1.0, 0.0), // front
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        // All vertices are on plane or front, so should be front only
        assert!(front.is_some());
        assert!(back.is_none());
    }

    #[test]
    fn polygon_edge_on_plane_with_back_vertices() {
        // Square where one edge lies on plane, opposite edge is behind
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.0, 0.0),  // on plane
            Point3::new(1.0, 0.0, 0.0),  // on plane
            Point3::new(1.0, -1.0, 0.0), // back
            Point3::new(0.0, -1.0, 0.0), // back
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        // On-plane vertices go to both, but front has no "front" vertices
        // This should classify as back since no vertices are strictly front
        // Actually, let's check the classification logic...
        // on_plane=2, front=0, back=2 => Classification::Back
        assert!(front.is_none());
        assert!(back.is_some());
    }

    #[test]
    fn polygon_two_vertices_on_plane_spanning() {
        // Hexagon with two non-adjacent vertices on the cutting plane
        // This ensures the split polygons have non-collinear first 3 vertices
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 1.0, 0.0),   // front
            Point3::new(1.0, 0.0, 0.0),   // on plane
            Point3::new(1.0, -1.0, 0.0),  // back
            Point3::new(0.0, -1.0, 0.0),  // back
            Point3::new(-1.0, 0.0, 0.0),  // on plane
            Point3::new(-1.0, 1.0, 0.0),  // front
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        let front = front.unwrap();
        let back = back.unwrap();

        // On-plane vertices should appear in both
        let on_plane_1 = Point3::new(1.0, 0.0, 0.0);
        let on_plane_2 = Point3::new(-1.0, 0.0, 0.0);

        assert!(
            front.vertices().contains(&on_plane_1),
            "On-plane vertex 1 should be in front polygon"
        );
        assert!(
            front.vertices().contains(&on_plane_2),
            "On-plane vertex 2 should be in front polygon"
        );
        assert!(
            back.vertices().contains(&on_plane_1),
            "On-plane vertex 1 should be in back polygon"
        );
        assert!(
            back.vertices().contains(&on_plane_2),
            "On-plane vertex 2 should be in back polygon"
        );

        // Verify exact vertex sequences (rotation-independent)
        let expected_front = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ];
        let expected_back = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
        ];

        assert!(
            same_polygon_vertices(front.vertices(), &expected_front),
            "Front polygon vertices mismatch.\nExpected: {expected_front:?}\nActual: {:?}",
            front.vertices()
        );
        assert!(
            same_polygon_vertices(back.vertices(), &expected_back),
            "Back polygon vertices mismatch.\nExpected: {expected_back:?}\nActual: {:?}",
            back.vertices()
        );
    }

    // =========================================================================
    // Triangle: Cuttable implementation tests
    // =========================================================================

    #[test]
    fn triangle_entirely_front() {
        let triangle = Triangle::new(
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(0.5, 1.5, 1.0),
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = triangle.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_none());
        assert_vertex_count(&front.unwrap(), 3);
    }

    #[test]
    fn triangle_entirely_back() {
        let triangle = Triangle::new(
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, -2.0, 0.0),
            Point3::new(0.5, -1.5, 1.0),
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = triangle.cut(&plane);

        assert!(front.is_none());
        assert!(back.is_some());
        assert_vertex_count(&back.unwrap(), 3);
    }

    #[test]
    fn triangle_coplanar() {
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 1.0),
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = triangle.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_none());
    }

    #[test]
    fn triangle_split_one_front_two_back() {
        let triangle = Triangle::new(
            Point3::new(0.0, 2.0, 0.0),   // front
            Point3::new(-1.0, -1.0, 0.0), // back
            Point3::new(1.0, -1.0, 0.0),  // back
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = triangle.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        // One front vertex + 2 intersections = triangle
        assert_vertex_count(&front.unwrap(), 3);
        // Two back vertices + 2 intersections = quad
        assert_vertex_count(&back.unwrap(), 4);
    }

    #[test]
    fn triangle_split_two_front_one_back() {
        let triangle = Triangle::new(
            Point3::new(-1.0, 1.0, 0.0), // front
            Point3::new(1.0, 1.0, 0.0),  // front
            Point3::new(0.0, -2.0, 0.0), // back
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = triangle.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        // Two front vertices + 2 intersections = quad
        assert_vertex_count(&front.unwrap(), 4);
        // One back vertex + 2 intersections = triangle
        assert_vertex_count(&back.unwrap(), 3);
    }

    // =========================================================================
    // Rectangle: Cuttable implementation tests
    // =========================================================================

    #[test]
    fn rectangle_entirely_front() {
        let rect = Rectangle::new(
            Point3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = rect.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_none());
        assert_vertex_count(&front.unwrap(), 4);
    }

    #[test]
    fn rectangle_entirely_back() {
        let rect = Rectangle::new(
            Point3::new(0.0, -2.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.5, 0.0),
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = rect.cut(&plane);

        assert!(front.is_none());
        assert!(back.is_some());
        assert_vertex_count(&back.unwrap(), 4);
    }

    #[test]
    fn rectangle_coplanar() {
        let rect = Rectangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = rect.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_none());
    }

    #[test]
    fn rectangle_split_horizontal() {
        // Rectangle spanning y = -1 to y = 1, cut at y = 0
        let rect = Rectangle::new(
            Point3::new(0.0, -1.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(0.0, 2.0, 0.0),
        );
        let plane = horizontal_plane(0.0);

        let (front, back) = rect.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        // Both halves should be quads
        assert_vertex_count(&front.unwrap(), 4);
        assert_vertex_count(&back.unwrap(), 4);
    }

    #[test]
    fn rectangle_split_diagonal() {
        // Rectangle cut diagonally
        let rect = Rectangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 2.0),
        );
        // Diagonal plane through the rectangle
        let plane = Plane3D::from_point_and_normal(
            Point3::new(1.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 1.0),
        );

        let (front, back) = rect.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        // Diagonal cut through a rectangle creates two triangles
        assert_vertex_count(&front.unwrap(), 3);
        assert_vertex_count(&back.unwrap(), 3);
    }

    #[test]
    fn rectangle_cut_off_corner() {
        // Rectangle with one corner cut off
        let rect = Rectangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 2.0),
        );
        // Plane that cuts off just one corner
        let plane = Plane3D::from_point_and_normal(
            Point3::new(1.5, 0.0, 1.5),
            Vector3::new(1.0, 0.0, 1.0),
        );

        let (front, back) = rect.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        let front = front.unwrap();
        let back = back.unwrap();

        // One side is a triangle (the cut corner)
        // Other side is a pentagon (original 4 - 1 + 2 intersections)
        let (small, large) = if front.len() < back.len() {
            (front, back)
        } else {
            (back, front)
        };

        assert_vertex_count(&small, 3);
        assert_vertex_count(&large, 5);
    }

    // =========================================================================
    // Different plane orientations
    // =========================================================================

    #[test]
    fn split_with_vertical_plane_x() {
        // Triangle split by a vertical plane (x = 0)
        let polygon = Polygon::new(vec![
            Point3::new(-1.0, 0.0, 0.0), // back (negative x)
            Point3::new(2.0, 0.0, 0.0),  // front (positive x)
            Point3::new(0.5, 1.0, 0.0),  // front
        ]);
        let plane = vertical_plane_x(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        assert_all_vertices_on_side(&front.unwrap(), &plane, PlaneSide::Front);
        assert_all_vertices_on_side(&back.unwrap(), &plane, PlaneSide::Back);
    }

    #[test]
    fn split_with_vertical_plane_z() {
        // Triangle split by a vertical plane (z = 0)
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.0, -1.0), // back (negative z)
            Point3::new(0.0, 0.0, 2.0),  // front (positive z)
            Point3::new(0.0, 1.0, 0.5),  // front
        ]);
        let plane = vertical_plane_z(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        assert_all_vertices_on_side(&front.unwrap(), &plane, PlaneSide::Front);
        assert_all_vertices_on_side(&back.unwrap(), &plane, PlaneSide::Back);
    }

    #[test]
    fn split_with_tilted_plane() {
        // Triangle split by a 45-degree tilted plane
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ]);
        // Plane at 45 degrees: x + y = 1
        let plane = Plane3D::from_point_and_normal(
            Point3::new(0.5, 0.5, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        );

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        assert_all_vertices_on_side(&front.unwrap(), &plane, PlaneSide::Front);
        assert_all_vertices_on_side(&back.unwrap(), &plane, PlaneSide::Back);
    }

    // =========================================================================
    // Geometric correctness tests
    // =========================================================================

    #[test]
    fn split_intersection_points_are_correct() {
        // Simple case: triangle from (-1, -1, 0) to (1, 1, 0) to (-1, 1, 0)
        // Split at y = 0
        let polygon = Polygon::new(vec![
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        let front = front.unwrap();
        let _back = back.unwrap();

        // Check that we can find the expected intersection points
        // Edge from (0, -1, 0) to (1, 1, 0) intersects at (0.5, 0, 0)
        // Edge from (0, -1, 0) to (-1, 1, 0) intersects at (-0.5, 0, 0)

        let has_intersection_1 = front.vertices().iter().any(|v| {
            approx_eq(v.x, 0.5, 1e-5) && approx_eq(v.y, 0.0, 1e-5)
        });
        let has_intersection_2 = front.vertices().iter().any(|v| {
            approx_eq(v.x, -0.5, 1e-5) && approx_eq(v.y, 0.0, 1e-5)
        });

        assert!(has_intersection_1, "Should have intersection at (0.5, 0, 0)");
        assert!(has_intersection_2, "Should have intersection at (-0.5, 0, 0)");
    }

    #[test]
    fn split_preserves_z_coordinates() {
        // Verify that splitting preserves Z coordinates correctly
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 1.0, 5.0),
            Point3::new(1.0, 1.0, 5.0),
            Point3::new(1.0, -1.0, 5.0),
            Point3::new(0.0, -1.0, 5.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        // All vertices should have z = 5
        for v in front.unwrap().vertices() {
            assert!(
                approx_eq(v.z, 5.0, 1e-5),
                "Z coordinate should be preserved: got {}",
                v.z
            );
        }
        for v in back.unwrap().vertices() {
            assert!(
                approx_eq(v.z, 5.0, 1e-5),
                "Z coordinate should be preserved: got {}",
                v.z
            );
        }
    }

    #[test]
    fn split_produces_valid_polygons() {
        // Verify that split polygons pass validity checks
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(2.0, 0.0, 1.0),
            Point3::new(0.0, -2.0, 0.0),
            Point3::new(-2.0, 0.0, -1.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        let front = front.unwrap();
        let back = back.unwrap();

        // Both polygons should have at least 3 vertices
        assert!(front.len() >= 3);
        assert!(back.len() >= 3);

        // Both polygons should be able to compute a normal (not degenerate)
        assert!(front.unit_normal().is_some());
        assert!(back.unit_normal().is_some());
    }

    // =========================================================================
    // Edge cases and boundary conditions
    // =========================================================================

    #[test]
    fn very_thin_slice() {
        // Polygon where the cut produces a very thin slice
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 0.001, 0.0),  // barely front
            Point3::new(1.0, 0.001, 0.0),  // barely front
            Point3::new(1.0, -1.0, 0.0),   // back
            Point3::new(0.0, -1.0, 0.0),   // back
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        // Should still produce valid results
        assert!(front.is_some());
        assert!(back.is_some());
    }

    #[test]
    fn large_polygon_many_vertices() {
        // Hexagon split in half
        let polygon = Polygon::new(vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.866, 0.0),
            Point3::new(-0.5, 0.866, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(-0.5, -0.866, 0.0),
            Point3::new(0.5, -0.866, 0.0),
        ]);
        let plane = horizontal_plane(0.0);

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        // Each half should be a quad (hexagon split horizontally)
        assert_vertex_count(&front.unwrap(), 4);
        assert_vertex_count(&back.unwrap(), 4);
    }

    #[test]
    fn polygon_with_offset_plane() {
        // Test with a plane not passing through origin
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(1.0, 5.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        ]);
        let plane = horizontal_plane(4.0); // y = 4

        let (front, back) = polygon.cut(&plane);

        assert!(front.is_some());
        assert!(back.is_some());

        assert_all_vertices_on_side(&front.unwrap(), &plane, PlaneSide::Front);
        assert_all_vertices_on_side(&back.unwrap(), &plane, PlaneSide::Back);
    }

    #[test]
    fn flipped_plane_reverses_classification() {
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.5, 2.0, 0.0),
        ]);

        let plane = horizontal_plane(0.0);
        let flipped_plane = plane.flipped();

        let (front1, back1) = polygon.cut(&plane);
        let (front2, back2) = polygon.cut(&flipped_plane);

        // Original: polygon is in front
        assert!(front1.is_some());
        assert!(back1.is_none());

        // Flipped: polygon is in back
        assert!(front2.is_none());
        assert!(back2.is_some());
    }

    // =========================================================================
    // Regression tests
    // =========================================================================

    #[test]
    fn split_does_not_produce_degenerate_polygons() {
        // Various configurations that could potentially produce degenerate results
        let test_cases = vec![
            // Triangle with apex at plane
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(-1.0, 1.0, 0.0),
            ],
            // Quad with edge on plane
            vec![
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(-1.0, 1.0, 0.0),
            ],
        ];

        let plane = horizontal_plane(0.0);

        for vertices in test_cases {
            let polygon = Polygon::new(vertices);
            let (front, back) = polygon.cut(&plane);

            if let Some(f) = front {
                assert!(f.len() >= 3, "Front polygon is degenerate");
            }
            if let Some(b) = back {
                assert!(b.len() >= 3, "Back polygon is degenerate");
            }
        }
    }

    #[test]
    fn consistent_results_with_equivalent_planes() {
        let polygon = Polygon::new(vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(-1.0, -1.0, 0.0),
        ]);

        // Same plane, created two different ways
        let plane1 = horizontal_plane(0.0);
        let plane2 = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);

        let (front1, back1) = polygon.cut(&plane1);
        let (front2, back2) = polygon.cut(&plane2);

        // Should produce identical results
        assert_eq!(front1.as_ref().map(|p| p.len()), front2.as_ref().map(|p| p.len()));
        assert_eq!(back1.as_ref().map(|p| p.len()), back2.as_ref().map(|p| p.len()));
    }
}
