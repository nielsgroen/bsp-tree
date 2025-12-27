# bsp-tree

A Binary Space Partitioning (BSP) tree implementation in Rust for 3D polygon management.
The BSP tree works with flat polygons (triangles, quads, etc.).

BSP trees recursively subdivide space using planes, enabling efficient spatial queries and correct back-to-front rendering of overlapping geometry without a depth buffer.

## Features

- **Geometric primitives**: `Polygon`, `Triangle`, `Rectangle`
- **Plane operations**: Classification of points and polygons relative to planes
- **Polygon splitting**: `Cuttable` trait for splitting geometry by planes
- **BSP tree construction**: Automatic partitioning with configurable plane selection
- **Ordered traversal**: Front-to-back or back-to-front traversal via `BspVisitor`

## Usage

```rust
use bsp_tree::{BspTree, Polygon};
use nalgebra::Point3;

// Create polygons
let poly = Polygon::new(vec![
    Point3::new(0.0, 0.0, 0.0),
    Point3::new(1.0, 0.0, 0.0),
    Point3::new(0.0, 1.0, 0.0),
]);

// Build the tree
let tree = BspTree::from_polygons(vec![poly]);

assert_eq!(tree.polygon_count(), 1);
```

## Traversal

```rust
use bsp_tree::{BspTree, BspVisitor, Polygon};
use nalgebra::Point3;

struct RenderVisitor;

impl BspVisitor for RenderVisitor {
    fn visit(&mut self, polygon: &Polygon) {
        // Render the polygon
    }
}

let tree = BspTree::from_polygons(polygons);
let eye = Point3::new(0.0, 0.0, 10.0);

// Back-to-front for correct transparency rendering
tree.traverse_back_to_front(eye, &mut RenderVisitor);
```

## Documentation

See the [GitHub repository](https://github.com/nielsgroen/bsp-tree) for full documentation, screenshots, and visualization examples.

## License

Apache-2.0 - See [LICENSE](https://github.com/nielsgroen/bsp-tree/blob/main/LICENSE) for details.
