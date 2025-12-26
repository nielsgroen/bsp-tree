//! BSP (Binary Space Partitioning) tree implementation.
//!
//! This crate provides a BSP tree for 3D polygon management, enabling efficient
//! spatial partitioning, ordered traversal, and polygon cutting operations.
//!
//! # Core Types
//!
//! - [`Polygon`], [`Triangle`], [`Rectangle`]: Geometric primitives
//! - [`Plane3D`]: 3D plane representation with classification operations
//! - [`Cuttable`]: Trait for splitting geometry by planes
//! - [`BspTree`]: The BSP tree container
//! - [`BspNode`]: Tree nodes holding splitting planes and coplanar polygons
//!
//! # Example
//!
//! ```
//! use bsp_tree::{BspTree, Polygon};
//! use nalgebra::Point3;
//!
//! // Create some polygons
//! let poly1 = Polygon::new(vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(1.0, 0.0, 0.0),
//!     Point3::new(0.0, 1.0, 0.0),
//! ]);
//!
//! // Build the tree
//! let tree = BspTree::from_polygons(vec![poly1]);
//!
//! assert_eq!(tree.polygon_count(), 1);
//! ```

pub mod bsp;
mod cuttable;
mod plane;
mod polygon;
mod rectangle;
mod triangle;

// Re-export BSP tree types at crate root for convenience
pub use bsp::{BspNode, BspTree, BspVisitor, FirstPolygon, PlaneSelector};

pub use cuttable::Cuttable;
pub use plane::{Classification, Plane3D, PlaneSide, PLANE_EPSILON};
pub use polygon::Polygon;
pub use rectangle::Rectangle;
pub use triangle::Triangle;
