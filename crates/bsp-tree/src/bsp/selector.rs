//! Plane selection strategies for BSP tree construction.
//!
//! The choice of splitting plane affects tree balance and the number of
//! polygon splits during construction. Different strategies offer different
//! trade-offs between build time and tree quality.

use crate::Polygon;

/// Strategy for selecting which polygon's plane to use for splitting.
///
/// The selected polygon's plane becomes the splitting plane for a BSP node.
/// Different strategies can optimize for:
/// - Build speed (simple selection)
/// - Tree balance (minimize depth)
/// - Minimal splits (preserve original polygons)
pub trait PlaneSelector {
    /// Select a polygon from the slice to use as the splitting plane.
    ///
    /// Returns `None` if the slice is empty.
    /// The returned reference must be to an element in the provided slice.
    fn select<'a>(&self, polygons: &'a [Polygon]) -> Option<&'a Polygon>;
}

/// Selects the first polygon in the list.
///
/// This is the simplest and fastest selector, but may produce unbalanced
/// trees depending on input order. Good for prototyping and when input
/// order is already randomized.
#[derive(Debug, Clone, Copy, Default)]
pub struct FirstPolygon;

impl PlaneSelector for FirstPolygon {
    fn select<'a>(&self, polygons: &'a [Polygon]) -> Option<&'a Polygon> {
        polygons.first()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    fn make_triangle(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> Polygon {
        Polygon::new(vec![
            Point3::new(a[0], a[1], a[2]),
            Point3::new(b[0], b[1], b[2]),
            Point3::new(c[0], c[1], c[2]),
        ])
    }

    #[test]
    fn first_polygon_empty_list() {
        let selector = FirstPolygon;
        let polygons: Vec<Polygon> = vec![];
        assert!(selector.select(&polygons).is_none());
    }

    #[test]
    fn first_polygon_single() {
        let selector = FirstPolygon;
        let poly = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let polygons = vec![poly.clone()];

        let selected = selector.select(&polygons);
        assert!(selected.is_some());
        assert_eq!(selected.unwrap(), &poly);
    }

    #[test]
    fn first_polygon_multiple() {
        let selector = FirstPolygon;
        let poly1 = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let poly2 = make_triangle([0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]);
        let polygons = vec![poly1.clone(), poly2];

        let selected = selector.select(&polygons);
        assert!(selected.is_some());
        assert_eq!(selected.unwrap(), &poly1);
    }
}
