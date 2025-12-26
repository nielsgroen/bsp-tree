//! BSP (Binary Space Partitioning) tree implementation.

mod plane;
mod polygon;
mod rectangle;
mod triangle;

pub use plane::{Classification, Plane3D, PlaneSide, PLANE_EPSILON};
pub use polygon::Polygon;
pub use rectangle::Rectangle;
pub use triangle::Triangle;
