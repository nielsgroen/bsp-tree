//! BSP tree container and construction.

use nalgebra::Point3;

use crate::{Classification, Cuttable, Polygon};

use super::node::{faces_same_direction, BspNode};
use super::selector::PlaneSelector;
use super::visitor::BspVisitor;

/// A Binary Space Partitioning tree for 3D polygons.
///
/// BSP trees recursively partition space using planes, enabling efficient
/// spatial queries and ordered traversal. Each node contains polygons that
/// are coplanar with its splitting plane, while non-coplanar polygons are
/// stored in front or back subtrees.
///
/// # Construction
///
/// Trees are built from a collection of polygons using a [`PlaneSelector`]
/// to choose splitting planes:
///
/// ```ignore
/// use bsp_tree::{BspTree, Polygon, FirstPolygon};
///
/// let polygons: Vec<Polygon> = /* ... */;
/// let tree = BspTree::build(polygons, &FirstPolygon);
/// ```
///
/// # Traversal
///
/// The tree supports front-to-back and back-to-front traversal relative to
/// a viewpoint, useful for painter's algorithm rendering:
///
/// ```ignore
/// tree.traverse_front_to_back(eye_position, &mut visitor);
/// ```
///
/// # Future: Insertion
///
/// The data structure supports insertion of polygons into an existing tree.
/// This operation will be implemented in a future version.
#[derive(Debug, Clone, Default)]
pub struct BspTree {
    root: Option<BspNode>,
}

impl BspTree {
    /// Creates an empty BSP tree.
    pub fn new() -> Self {
        Self { root: None }
    }

    /// Builds a BSP tree from a collection of polygons.
    ///
    /// Uses the provided [`PlaneSelector`] to choose splitting planes during
    /// construction. Polygons that span a splitting plane are automatically
    /// split using the [`Cuttable`] trait.
    ///
    /// Returns an empty tree if the input is empty.
    pub fn build<S: PlaneSelector>(polygons: Vec<Polygon>, selector: &S) -> Self {
        Self {
            root: build_node(polygons, selector),
        }
    }

    /// Builds a BSP tree using the default plane selector ([`FirstPolygon`]).
    pub fn from_polygons(polygons: Vec<Polygon>) -> Self {
        use super::selector::FirstPolygon;
        Self::build(polygons, &FirstPolygon)
    }

    /// Returns `true` if the tree contains no polygons.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.root.is_none()
    }

    /// Returns a reference to the root node, if any.
    #[inline]
    pub fn root(&self) -> Option<&BspNode> {
        self.root.as_ref()
    }

    /// Returns a mutable reference to the root node, if any.
    ///
    /// This is primarily for future insert operations.
    #[inline]
    pub fn root_mut(&mut self) -> Option<&mut BspNode> {
        self.root.as_mut()
    }

    /// Returns the total number of polygons in the tree.
    pub fn polygon_count(&self) -> usize {
        self.root.as_ref().map_or(0, |n| n.polygon_count())
    }

    /// Returns the maximum depth of the tree (0 for empty tree).
    pub fn depth(&self) -> usize {
        self.root.as_ref().map_or(0, |n| n.depth())
    }

    /// Traverses the tree front-to-back relative to the given viewpoint.
    ///
    /// This ordering is useful for rendering with the painter's algorithm,
    /// where front polygons should be drawn last (on top).
    ///
    /// The visitor's `visit` method is called for each group of coplanar
    /// polygons, in front-to-back order.
    pub fn traverse_front_to_back<V: BspVisitor>(&self, eye: Point3<f32>, visitor: &mut V) {
        if let Some(ref root) = self.root {
            traverse_front_to_back_node(root, eye, visitor);
        }
    }

    /// Traverses the tree back-to-front relative to the given viewpoint.
    ///
    /// This is the reverse of front-to-back traversal, useful when you need
    /// back polygons first (e.g., for certain transparency effects).
    pub fn traverse_back_to_front<V: BspVisitor>(&self, eye: Point3<f32>, visitor: &mut V) {
        if let Some(ref root) = self.root {
            traverse_back_to_front_node(root, eye, visitor);
        }
    }

    /// Collects all polygons in the tree into a vector.
    ///
    /// The order of polygons is not guaranteed.
    pub fn collect_polygons(&self) -> Vec<Polygon> {
        let mut result = Vec::with_capacity(self.polygon_count());
        collect_polygons_recursive(self.root.as_ref(), &mut result);
        result
    }

    // TODO: Future insert operation
    // pub fn insert(&mut self, polygon: Polygon) { ... }
}

/// Recursively builds a BSP node from a list of polygons.
fn build_node<S: PlaneSelector>(mut polygons: Vec<Polygon>, selector: &S) -> Option<BspNode> {
    if polygons.is_empty() {
        return None;
    }

    // Select the splitting polygon and derive the plane
    let splitter_idx = polygons
        .iter()
        .position(|p| Some(p) == selector.select(&polygons))?;

    let splitter = polygons.swap_remove(splitter_idx);
    let plane = splitter.plane();

    // Initialize lists
    let mut coplanar_front = Vec::new();
    let mut coplanar_back = Vec::new();
    let mut front_list = Vec::new();
    let mut back_list = Vec::new();

    // The splitter itself is coplanar - determine its facing
    if faces_same_direction(&splitter, &plane) {
        coplanar_front.push(splitter);
    } else {
        coplanar_back.push(splitter);
    }

    // Classify and partition remaining polygons
    for polygon in polygons {
        match polygon.classify(&plane) {
            Classification::Front => {
                front_list.push(polygon);
            }
            Classification::Back => {
                back_list.push(polygon);
            }
            Classification::Coplanar => {
                if faces_same_direction(&polygon, &plane) {
                    coplanar_front.push(polygon);
                } else {
                    coplanar_back.push(polygon);
                }
            }
            Classification::Spanning => {
                let (front_part, back_part) = polygon.cut(&plane);
                if let Some(f) = front_part {
                    front_list.push(f);
                }
                if let Some(b) = back_part {
                    back_list.push(b);
                }
            }
        }
    }

    // Build the node with children
    let mut node = BspNode::with_coplanar(plane, coplanar_front, coplanar_back);
    node.set_front(build_node(front_list, selector));
    node.set_back(build_node(back_list, selector));

    Some(node)
}

/// Traverses a node subtree front-to-back.
fn traverse_front_to_back_node<V: BspVisitor>(node: &BspNode, eye: Point3<f32>, visitor: &mut V) {
    let side = node.plane().classify_point(eye);

    // Collect coplanar polygons for visiting
    let coplanar: Vec<Polygon> = node.all_coplanar().cloned().collect();

    match side {
        crate::PlaneSide::Front | crate::PlaneSide::OnPlane => {
            // Eye is in front: front subtree is closer
            if let Some(front) = node.front() {
                traverse_front_to_back_node(front, eye, visitor);
            }
            if !coplanar.is_empty() {
                visitor.visit(&coplanar);
            }
            if let Some(back) = node.back() {
                traverse_front_to_back_node(back, eye, visitor);
            }
        }
        crate::PlaneSide::Back => {
            // Eye is behind: back subtree is closer
            if let Some(back) = node.back() {
                traverse_front_to_back_node(back, eye, visitor);
            }
            if !coplanar.is_empty() {
                visitor.visit(&coplanar);
            }
            if let Some(front) = node.front() {
                traverse_front_to_back_node(front, eye, visitor);
            }
        }
    }
}

/// Traverses a node subtree back-to-front.
fn traverse_back_to_front_node<V: BspVisitor>(node: &BspNode, eye: Point3<f32>, visitor: &mut V) {
    let side = node.plane().classify_point(eye);

    let coplanar: Vec<Polygon> = node.all_coplanar().cloned().collect();

    match side {
        crate::PlaneSide::Front | crate::PlaneSide::OnPlane => {
            // Eye is in front: back subtree is farther
            if let Some(back) = node.back() {
                traverse_back_to_front_node(back, eye, visitor);
            }
            if !coplanar.is_empty() {
                visitor.visit(&coplanar);
            }
            if let Some(front) = node.front() {
                traverse_back_to_front_node(front, eye, visitor);
            }
        }
        crate::PlaneSide::Back => {
            // Eye is behind: front subtree is farther
            if let Some(front) = node.front() {
                traverse_back_to_front_node(front, eye, visitor);
            }
            if !coplanar.is_empty() {
                visitor.visit(&coplanar);
            }
            if let Some(back) = node.back() {
                traverse_back_to_front_node(back, eye, visitor);
            }
        }
    }
}

/// Recursively collects all polygons from a node subtree.
fn collect_polygons_recursive(node: Option<&BspNode>, result: &mut Vec<Polygon>) {
    if let Some(n) = node {
        result.extend(n.all_coplanar().cloned());
        collect_polygons_recursive(n.front(), result);
        collect_polygons_recursive(n.back(), result);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bsp::visitor::CollectingVisitor;
    use nalgebra::Point3;

    fn make_triangle(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> Polygon {
        Polygon::new(vec![
            Point3::new(a[0], a[1], a[2]),
            Point3::new(b[0], b[1], b[2]),
            Point3::new(c[0], c[1], c[2]),
        ])
    }

    #[test]
    fn empty_tree() {
        let tree = BspTree::new();
        assert!(tree.is_empty());
        assert_eq!(tree.polygon_count(), 0);
        assert_eq!(tree.depth(), 0);
    }

    #[test]
    fn build_empty() {
        let tree = BspTree::from_polygons(vec![]);
        assert!(tree.is_empty());
    }

    #[test]
    fn build_single_polygon() {
        let poly = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let tree = BspTree::from_polygons(vec![poly]);

        assert!(!tree.is_empty());
        assert_eq!(tree.polygon_count(), 1);
        assert_eq!(tree.depth(), 1);
    }

    #[test]
    fn build_two_parallel_polygons() {
        // Two triangles on parallel planes (not coplanar)
        let poly1 = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let poly2 = make_triangle([0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]);

        let tree = BspTree::from_polygons(vec![poly1, poly2]);

        assert_eq!(tree.polygon_count(), 2);
        // One should be in front or back of the other
        assert!(tree.depth() >= 1);
    }

    #[test]
    fn build_coplanar_same_facing() {
        // Two triangles on the same plane, same winding
        let poly1 = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let poly2 = make_triangle([1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0]);

        let tree = BspTree::from_polygons(vec![poly1, poly2]);

        assert_eq!(tree.polygon_count(), 2);
        // Both coplanar, should be in same node
        assert_eq!(tree.depth(), 1);

        let root = tree.root().unwrap();
        assert_eq!(root.coplanar_count(), 2);
    }

    #[test]
    fn build_spanning_polygon_gets_split() {
        // First polygon on Y=0 plane
        let splitter = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);

        // Second polygon spans the Y=0 plane
        let spanning = make_triangle([-0.5, -1.0, 0.5], [0.5, 1.0, 0.5], [0.5, -1.0, 0.5]);

        let tree = BspTree::from_polygons(vec![splitter, spanning]);

        // Original was 2 polygons, but spanning got split into 2
        // So we should have 3 total
        assert_eq!(tree.polygon_count(), 3);
    }

    #[test]
    fn traverse_front_to_back_single() {
        let poly = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let tree = BspTree::from_polygons(vec![poly.clone()]);

        let mut visitor = CollectingVisitor::new();
        tree.traverse_front_to_back(Point3::new(0.0, 0.0, 10.0), &mut visitor);

        assert_eq!(visitor.polygons().len(), 1);
    }

    #[test]
    fn traverse_front_to_back_ordering() {
        // Create two polygons at different Z depths
        // poly_near at z=1, poly_far at z=-1
        let poly_near = make_triangle([0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]);
        let poly_far = make_triangle([0.0, 0.0, -1.0], [1.0, 0.0, -1.0], [0.0, 1.0, -1.0]);

        let tree = BspTree::from_polygons(vec![poly_far.clone(), poly_near.clone()]);

        // Eye at z=10 looking toward origin
        let mut visitor = CollectingVisitor::new();
        tree.traverse_front_to_back(Point3::new(0.5, 0.5, 10.0), &mut visitor);

        let collected = visitor.into_polygons();
        assert_eq!(collected.len(), 2);

        // Front-to-back from z=10: poly_near (z=1) should come first
        // (it's in front from the eye's perspective)
        let first_z = collected[0].centroid().z;
        let second_z = collected[1].centroid().z;
        assert!(
            first_z > second_z,
            "Expected front-to-back order: first_z={} should be > second_z={}",
            first_z,
            second_z
        );
    }

    #[test]
    fn traverse_back_to_front_ordering() {
        let poly_near = make_triangle([0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]);
        let poly_far = make_triangle([0.0, 0.0, -1.0], [1.0, 0.0, -1.0], [0.0, 1.0, -1.0]);

        let tree = BspTree::from_polygons(vec![poly_far.clone(), poly_near.clone()]);

        let mut visitor = CollectingVisitor::new();
        tree.traverse_back_to_front(Point3::new(0.5, 0.5, 10.0), &mut visitor);

        let collected = visitor.into_polygons();
        assert_eq!(collected.len(), 2);

        // Back-to-front: poly_far (z=-1) should come first
        let first_z = collected[0].centroid().z;
        let second_z = collected[1].centroid().z;
        assert!(
            first_z < second_z,
            "Expected back-to-front order: first_z={} should be < second_z={}",
            first_z,
            second_z
        );
    }

    #[test]
    fn collect_polygons() {
        let poly1 = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let poly2 = make_triangle([0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]);
        let poly3 = make_triangle([0.0, 0.0, 2.0], [1.0, 0.0, 2.0], [0.0, 1.0, 2.0]);

        let tree = BspTree::from_polygons(vec![poly1, poly2, poly3]);
        let collected = tree.collect_polygons();

        assert_eq!(collected.len(), 3);
    }
}
