//! Binary Space Partitioning tree for 3D polygon management.
//!
//! This module provides a BSP tree implementation that recursively partitions
//! 3D space using planes derived from input polygons. The tree enables:
//!
//! - Efficient front-to-back and back-to-front traversal for rendering
//! - Spatial queries based on viewpoint position
//! - Future: polygon insertion and CSG operations
//!
//! # Example
//!
//! ```ignore
//! use bsp_tree::{BspTree, Polygon, FirstPolygon};
//! use bsp_tree::bsp::{CollectingVisitor};
//! use nalgebra::Point3;
//!
//! // Build a tree from polygons
//! let polygons: Vec<Polygon> = /* create polygons */;
//! let tree = BspTree::from_polygons(polygons);
//!
//! // Traverse front-to-back for rendering
//! let eye = Point3::new(0.0, 0.0, 10.0);  // The location of the viewer
//! let mut visitor = CollectingVisitor::new();
//! tree.traverse_front_to_back(eye, &mut visitor);
//!
//! // Get polygons in render order
//! let ordered_polygons = visitor.into_polygons();
//! ```
//!
//! # Architecture
//!
//! - [`BspTree`]: The main container holding the root node
//! - [`BspNode`]: Internal nodes storing a splitting plane and coplanar polygons
//! - [`PlaneSelector`]: Strategy trait for choosing splitting planes
//! - [`BspVisitor`]: Visitor trait for custom traversal behavior

mod node;
mod selector;
mod tree;
mod visitor;

// Re-export main types
pub use node::{faces_same_direction, BspNode};
pub use selector::{FirstPolygon, PlaneSelector};
pub use tree::BspTree;
pub use visitor::{BspVisitor, CollectingVisitor, FnVisitor};
