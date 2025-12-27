//! BSP tree node implementation.

use crate::{Plane3D, Polygon};

/// A node in the BSP tree.
///
/// Each node partitions space using a splitting plane and stores polygons
/// that are coplanar with that plane. Polygons on the front or back of the
/// plane are stored in the respective child subtrees.
///
/// # Coplanar Polygon Storage
///
/// Coplanar polygons are separated by their facing direction relative to
/// the splitting plane's normal:
/// - `coplanar_front`: polygons whose normal points the same direction as the plane normal
/// - `coplanar_back`: polygons whose normal points opposite to the plane normal
///
/// This distinction is important for CSG operations where polygon
/// facing determines inside/outside classification.
#[derive(Debug, Clone)]
pub struct BspNode {
    /// The splitting plane for this node.
    plane: Plane3D,

    /// Polygons coplanar with the plane, facing the SAME direction as the plane normal.
    coplanar_front: Vec<Polygon>,

    /// Polygons coplanar with the plane, facing the OPPOSITE direction as the plane normal.
    coplanar_back: Vec<Polygon>,

    /// Subtree containing polygons in FRONT of the splitting plane.
    front: Option<Box<BspNode>>,

    /// Subtree containing polygons BEHIND the splitting plane.
    back: Option<Box<BspNode>>,
}

impl BspNode {
    /// Creates a new BSP node with the given splitting plane.
    ///
    /// The node starts with no coplanar polygons and no children.
    pub fn new(plane: Plane3D) -> Self {
        Self {
            plane,
            coplanar_front: Vec::new(),
            coplanar_back: Vec::new(),
            front: None,
            back: None,
        }
    }

    /// Creates a new BSP node with a splitting plane and initial coplanar polygons.
    pub fn with_coplanar(
        plane: Plane3D,
        coplanar_front: Vec<Polygon>,
        coplanar_back: Vec<Polygon>,
    ) -> Self {
        Self {
            plane,
            coplanar_front,
            coplanar_back,
            front: None,
            back: None,
        }
    }

    /// Returns a reference to the splitting plane.
    #[inline]
    pub fn plane(&self) -> &Plane3D {
        &self.plane
    }

    /// Returns coplanar polygons facing the same direction as the plane normal.
    #[inline]
    pub fn coplanar_front(&self) -> &[Polygon] {
        &self.coplanar_front
    }

    /// Returns coplanar polygons facing opposite to the plane normal.
    #[inline]
    pub fn coplanar_back(&self) -> &[Polygon] {
        &self.coplanar_back
    }

    /// Returns all coplanar polygons at this node (both front and back facing).
    pub fn all_coplanar(&self) -> impl Iterator<Item = &Polygon> {
        self.coplanar_front.iter().chain(self.coplanar_back.iter())
    }

    /// Returns the number of coplanar polygons at this node.
    pub fn coplanar_count(&self) -> usize {
        self.coplanar_front.len() + self.coplanar_back.len()
    }

    /// Returns a reference to the front child subtree.
    #[inline]
    pub fn front(&self) -> Option<&BspNode> {
        self.front.as_deref()
    }

    /// Returns a reference to the back child subtree.
    #[inline]
    pub fn back(&self) -> Option<&BspNode> {
        self.back.as_deref()
    }

    /// Returns a mutable reference to the front child subtree.
    #[inline]
    pub fn front_mut(&mut self) -> Option<&mut BspNode> {
        self.front.as_deref_mut()
    }

    /// Returns a mutable reference to the back child subtree.
    #[inline]
    pub fn back_mut(&mut self) -> Option<&mut BspNode> {
        self.back.as_deref_mut()
    }

    /// Sets the front child subtree.
    #[inline]
    pub fn set_front(&mut self, node: Option<BspNode>) {
        self.front = node.map(Box::new);
    }

    /// Sets the back child subtree.
    #[inline]
    pub fn set_back(&mut self, node: Option<BspNode>) {
        self.back = node.map(Box::new);
    }

    /// Adds a polygon to the coplanar front list.
    #[inline]
    pub fn add_coplanar_front(&mut self, polygon: Polygon) {
        self.coplanar_front.push(polygon);
    }

    /// Adds a polygon to the coplanar back list.
    #[inline]
    pub fn add_coplanar_back(&mut self, polygon: Polygon) {
        self.coplanar_back.push(polygon);
    }

    /// Checks if this node has any children.
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.front.is_none() && self.back.is_none()
    }

    /// Returns the total number of polygons in this subtree (including all descendants).
    pub fn polygon_count(&self) -> usize {
        let mut count = self.coplanar_count();

        if let Some(ref front) = self.front {
            count += front.polygon_count();
        }
        if let Some(ref back) = self.back {
            count += back.polygon_count();
        }

        count
    }

    /// Returns the depth of this subtree (1 for a leaf node).
    pub fn depth(&self) -> usize {
        let front_depth = self.front.as_ref().map_or(0, |n| n.depth());
        let back_depth = self.back.as_ref().map_or(0, |n| n.depth());
        1 + front_depth.max(back_depth)
    }
}

/// Determines if a polygon faces the same direction as a plane.
///
/// Compares the polygon's normal to the plane's normal using the dot product.
/// Returns `true` if the normals point in roughly the same direction (dot > 0).
///
/// # Panics
///
/// Panics if the polygon has a degenerate (zero-length) normal.
#[inline]
pub fn faces_same_direction(polygon: &Polygon, plane: &Plane3D) -> bool {
    let poly_normal = polygon
        .unit_normal()
        .expect("Polygon must have a valid normal for BSP operations");
    poly_normal.dot(&plane.normal()) > 0.0
}


#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    fn make_triangle(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> Polygon {
        Polygon::new(vec![
            Point3::new(a[0], a[1], a[2]),
            Point3::new(b[0], b[1], b[2]),
            Point3::new(c[0], c[1], c[2]),
        ])
    }

    #[test]
    fn new_node_is_empty_leaf() {
        let plane = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);
        let node = BspNode::new(plane);

        assert!(node.is_leaf());
        assert_eq!(node.coplanar_count(), 0);
        assert_eq!(node.polygon_count(), 0);
        assert_eq!(node.depth(), 1);
    }

    #[test]
    fn with_coplanar_stores_polygons() {
        let plane = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);
        let poly1 = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let poly2 = make_triangle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);

        let node = BspNode::with_coplanar(plane, vec![poly1], vec![poly2]);

        assert_eq!(node.coplanar_front().len(), 1);
        assert_eq!(node.coplanar_back().len(), 1);
        assert_eq!(node.coplanar_count(), 2);
    }

    #[test]
    fn set_children_updates_leaf_status() {
        let plane = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);
        let mut node = BspNode::new(plane.clone());

        assert!(node.is_leaf());

        node.set_front(Some(BspNode::new(plane.clone())));
        assert!(!node.is_leaf());

        node.set_front(None);
        assert!(node.is_leaf());

        node.set_back(Some(BspNode::new(plane)));
        assert!(!node.is_leaf());
    }

    #[test]
    fn depth_calculation() {
        let plane = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);
        let mut root = BspNode::new(plane.clone());
        assert_eq!(root.depth(), 1);

        let mut front = BspNode::new(plane.clone());
        front.set_front(Some(BspNode::new(plane.clone())));
        root.set_front(Some(front));

        // root -> front -> front (depth 3)
        assert_eq!(root.depth(), 3);

        root.set_back(Some(BspNode::new(plane)));
        // Still depth 3 (front branch is deeper)
        assert_eq!(root.depth(), 3);
    }

    #[test]
    fn polygon_count_recursive() {
        let plane = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);
        let poly = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);

        let mut root = BspNode::with_coplanar(plane.clone(), vec![poly.clone()], vec![]);
        assert_eq!(root.polygon_count(), 1);

        let front = BspNode::with_coplanar(plane.clone(), vec![poly.clone(), poly.clone()], vec![]);
        let back = BspNode::with_coplanar(plane, vec![], vec![poly]);
        root.set_front(Some(front));
        root.set_back(Some(back));

        assert_eq!(root.polygon_count(), 4);
    }

    #[test]
    fn faces_same_direction_positive() {
        // Polygon on XZ plane with normal pointing up (+Y)
        let poly = make_triangle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let plane = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);

        // Polygon normal is -Y (due to winding), plane normal is +Y
        // Let's check...
        let poly_normal = poly.unit_normal().unwrap();
        // Cross product of (1,0,0) x (0,0,1) = (0,-1,0)
        assert!(poly_normal.y < 0.0);

        // So they face opposite directions
        assert!(!faces_same_direction(&poly, &plane));
    }

    #[test]
    fn faces_same_direction_negative() {
        // Polygon with normal pointing up (+Y)
        let poly = make_triangle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
        let plane = Plane3D::new(Vector3::new(0.0, 1.0, 0.0), 0.0);

        // Cross product of (0,0,1) x (1,0,0) = (0,1,0) which is +Y
        let poly_normal = poly.unit_normal().unwrap();
        assert!(poly_normal.y > 0.0);

        // Same direction as plane normal
        assert!(faces_same_direction(&poly, &plane));
    }
}
