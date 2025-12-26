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
