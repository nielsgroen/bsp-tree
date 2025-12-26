//! Visitor pattern for BSP tree traversal.
//!
//! Visitors allow custom processing of polygons during tree traversal
//! without coupling traversal logic to specific use cases.

use crate::Polygon;

/// Visitor for processing polygons during BSP tree traversal.
///
/// Implement this trait to define custom behavior when traversing the tree.
/// Common uses include:
/// - Rendering (painter's algorithm)
/// - Collecting polygons in sorted order
/// - Computing visibility
pub trait BspVisitor {
    /// Called for each group of coplanar polygons during traversal.
    ///
    /// The polygons passed to this method are all coplanar with each other
    /// and belong to the same BSP node.
    fn visit(&mut self, polygons: &[Polygon]);
}

/// A simple visitor that collects all visited polygons.
#[derive(Debug, Default)]
pub struct CollectingVisitor {
    collected: Vec<Polygon>,
}

impl CollectingVisitor {
    /// Creates a new empty collecting visitor.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the collected polygons.
    pub fn into_polygons(self) -> Vec<Polygon> {
        self.collected
    }

    /// Returns a reference to the collected polygons.
    pub fn polygons(&self) -> &[Polygon] {
        &self.collected
    }
}

impl BspVisitor for CollectingVisitor {
    fn visit(&mut self, polygons: &[Polygon]) {
        self.collected.extend(polygons.iter().cloned());
    }
}

/// A visitor that calls a closure for each polygon group.
pub struct FnVisitor<F>
where
    F: FnMut(&[Polygon]),
{
    func: F,
}

impl<F> FnVisitor<F>
where
    F: FnMut(&[Polygon]),
{
    /// Creates a new visitor from a closure.
    pub fn new(func: F) -> Self {
        Self { func }
    }
}

impl<F> BspVisitor for FnVisitor<F>
where
    F: FnMut(&[Polygon]),
{
    fn visit(&mut self, polygons: &[Polygon]) {
        (self.func)(polygons);
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
    fn collecting_visitor_empty() {
        let visitor = CollectingVisitor::new();
        assert!(visitor.polygons().is_empty());
    }

    #[test]
    fn collecting_visitor_collects() {
        let mut visitor = CollectingVisitor::new();
        let poly1 = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let poly2 = make_triangle([0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]);

        visitor.visit(&[poly1.clone()]);
        visitor.visit(&[poly2.clone()]);

        let collected = visitor.into_polygons();
        assert_eq!(collected.len(), 2);
        assert_eq!(collected[0], poly1);
        assert_eq!(collected[1], poly2);
    }

    #[test]
    fn fn_visitor_calls_closure() {
        let mut count = 0;
        {
            let mut visitor = FnVisitor::new(|polys: &[Polygon]| {
                count += polys.len();
            });

            let poly = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
            visitor.visit(&[poly.clone(), poly]);
        }
        assert_eq!(count, 2);
    }
}
