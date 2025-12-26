//! BSP (Binary Space Partitioning) tree implementation.

mod plane;
mod rectangle;
mod triangle;

pub use plane::{Plane3D, PlaneSide, PLANE_EPSILON};
pub use rectangle::{Rectangle, RectangleClassification};
pub use triangle::{Triangle, TriangleClassification};
