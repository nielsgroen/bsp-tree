//! Shared visualization utilities for BSP tree examples.

use std::hash::{Hash, Hasher};

use bsp_tree::{BspVisitor, Plane3D, Polygon, Rectangle};
use macroquad::models::{draw_mesh, Mesh, Vertex};
use macroquad::prelude::*;
use nalgebra::{Point3, Rotation3, Vector3};

pub mod navigator;
pub use navigator::TreeNavigator;

/// Generates a deterministic color from a polygon's vertices using hashing.
/// This ensures split polygons get consistent colors across frames.
pub fn polygon_color(polygon: &Polygon) -> Color {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    for v in polygon.vertices() {
        v.x.to_bits().hash(&mut hasher);
        v.y.to_bits().hash(&mut hasher);
        v.z.to_bits().hash(&mut hasher);
    }
    let hash = hasher.finish();

    // Extract RGB from hash bytes
    let r = ((hash >> 16) & 0xFF) as u8;
    let g = ((hash >> 8) & 0xFF) as u8;
    let b = (hash & 0xFF) as u8;

    // Ensure colors aren't too dark by adding a minimum brightness
    let r = r.max(40);
    let g = g.max(40);
    let b = b.max(40);

    Color::from_rgba(r, g, b, 255)
}

/// Draws a single polygon by triangulating it (fan triangulation) using a Mesh.
pub fn draw_polygon(polygon: &Polygon) {
    let verts = polygon.vertices();
    if verts.len() < 3 {
        return;
    }

    let color = polygon_color(polygon);

    // Convert nalgebra points to macroquad Vertices
    let mesh_vertices: Vec<Vertex> = verts
        .iter()
        .map(|p| Vertex::new2(vec3(p.x, p.y, p.z), vec2(0.0, 0.0), color))
        .collect();

    // Fan triangulation: vertex 0 connects to all edges
    let mut indices: Vec<u16> = Vec::with_capacity((verts.len() - 2) * 3);
    for i in 1..verts.len() - 1 {
        indices.push(0);
        indices.push(i as u16);
        indices.push((i + 1) as u16);
    }

    let mesh = Mesh {
        vertices: mesh_vertices,
        indices,
        texture: None,
    };

    draw_mesh(&mesh);
}

/// Visitor that renders polygons using macroquad's 3D drawing.
pub struct RenderVisitor;

impl BspVisitor for RenderVisitor {
    fn visit(&mut self, polygons: &[Polygon]) {
        for polygon in polygons {
            draw_polygon(polygon);
        }
    }
}

/// Generates the 6 face polygons of an axis-aligned cube.
pub fn generate_cube_polygons(center: Point3<f32>, size: f32) -> Vec<Polygon> {
    let half = size / 2.0;

    // 8 corners of the cube
    let corners = [
        center + Vector3::new(-half, -half, -half), // 0: left-bottom-back
        center + Vector3::new(half, -half, -half),  // 1: right-bottom-back
        center + Vector3::new(half, half, -half),   // 2: right-top-back
        center + Vector3::new(-half, half, -half),  // 3: left-top-back
        center + Vector3::new(-half, -half, half),  // 4: left-bottom-front
        center + Vector3::new(half, -half, half),   // 5: right-bottom-front
        center + Vector3::new(half, half, half),    // 6: right-top-front
        center + Vector3::new(-half, half, half),   // 7: left-top-front
    ];

    // 6 faces with counter-clockwise winding (viewed from outside)
    let faces: [[usize; 4]; 6] = [
        [4, 5, 6, 7], // front (+Z)
        [1, 0, 3, 2], // back (-Z)
        [0, 4, 7, 3], // left (-X)
        [5, 1, 2, 6], // right (+X)
        [7, 6, 2, 3], // top (+Y)
        [0, 1, 5, 4], // bottom (-Y)
    ];

    faces
        .iter()
        .map(|indices| {
            Rectangle::from_corners(
                corners[indices[0]],
                corners[indices[1]],
                corners[indices[2]],
                corners[indices[3]],
            )
            .into()
        })
        .collect()
}

/// Creates a coplanar quad by projecting the 4th vertex onto the plane.
/// This fixes floating-point precision issues that occur after rotation.
fn make_coplanar_quad(
    p0: Point3<f32>,
    p1: Point3<f32>,
    p2: Point3<f32>,
    p3: Point3<f32>,
) -> Polygon {
    let plane = Plane3D::from_three_points(p0, p1, p2);
    let p3_projected = plane.project_point(p3);
    Polygon::new(vec![p0, p1, p2, p3_projected])
}

/// Generates the 6 face polygons of a rotated cube.
pub fn generate_rotated_cube(
    center: Point3<f32>,
    size: f32,
    rotation: &Rotation3<f32>,
) -> Vec<Polygon> {
    let half = size / 2.0;

    let unit_corners = [
        Vector3::new(-half, -half, -half),
        Vector3::new(half, -half, -half),
        Vector3::new(half, half, -half),
        Vector3::new(-half, half, -half),
        Vector3::new(-half, -half, half),
        Vector3::new(half, -half, half),
        Vector3::new(half, half, half),
        Vector3::new(-half, half, half),
    ];

    let corners: Vec<Point3<f32>> = unit_corners
        .iter()
        .map(|v| center + rotation * v)
        .collect();

    let faces: [[usize; 4]; 6] = [
        [4, 5, 6, 7], // front (+Z)
        [1, 0, 3, 2], // back (-Z)
        [0, 4, 7, 3], // left (-X)
        [5, 1, 2, 6], // right (+X)
        [7, 6, 2, 3], // top (+Y)
        [0, 1, 5, 4], // bottom (-Y)
    ];

    faces
        .iter()
        .map(|indices| {
            make_coplanar_quad(
                corners[indices[0]],
                corners[indices[1]],
                corners[indices[2]],
                corners[indices[3]],
            )
        })
        .collect()
}

/// Simple orbit camera for 3D scene navigation.
pub struct OrbitCamera {
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub target: Vec3,
    /// Multiplier for scroll wheel zoom
    pub zoom_speed: f32,
    /// Minimum distance from target
    pub min_distance: f32,
    /// Maximum distance from target
    pub max_distance: f32,
}

impl OrbitCamera {
    /// Creates a new orbit camera with the given configuration.
    pub fn new(distance: f32, yaw: f32, pitch: f32) -> Self {
        Self {
            distance,
            yaw,
            pitch,
            target: vec3(0.0, 0.0, 0.0),
            zoom_speed: 5.0,
            min_distance: 10.0,
            max_distance: 200.0,
        }
    }

    /// Sets the zoom configuration (speed and distance limits).
    pub fn with_zoom(mut self, speed: f32, min: f32, max: f32) -> Self {
        self.zoom_speed = speed;
        self.min_distance = min;
        self.max_distance = max;
        self
    }

    /// Sets the camera target point.
    pub fn with_target(mut self, target: Vec3) -> Self {
        self.target = target;
        self
    }

    /// Updates camera state from user input (mouse drag, scroll, arrow keys).
    pub fn update(&mut self) {
        // Mouse drag for rotation
        if is_mouse_button_down(MouseButton::Left) {
            let delta = mouse_delta_position();
            self.yaw -= delta.x * 2.0;
            self.pitch -= delta.y * 2.0;
        }

        // Clamp pitch to avoid gimbal lock
        self.pitch = self.pitch.clamp(-1.5, 1.5);

        // Mouse wheel for zoom
        let scroll = mouse_wheel().1;
        self.distance -= scroll * self.zoom_speed;
        self.distance = self.distance.clamp(self.min_distance, self.max_distance);

        // Arrow keys for rotation
        if is_key_down(KeyCode::Left) {
            self.yaw += 0.02;
        }
        if is_key_down(KeyCode::Right) {
            self.yaw -= 0.02;
        }
        if is_key_down(KeyCode::Up) {
            self.pitch += 0.02;
        }
        if is_key_down(KeyCode::Down) {
            self.pitch -= 0.02;
        }
    }

    /// Returns the camera's world position.
    pub fn position(&self) -> Vec3 {
        let x = self.distance * self.pitch.cos() * self.yaw.sin();
        let y = self.distance * self.pitch.sin();
        let z = self.distance * self.pitch.cos() * self.yaw.cos();
        self.target + vec3(x, y, z)
    }

    /// Converts to macroquad's Camera3D for rendering.
    pub fn to_camera3d(&self) -> Camera3D {
        Camera3D {
            position: self.position(),
            up: vec3(0.0, 1.0, 0.0),
            target: self.target,
            ..Default::default()
        }
    }

    /// Returns the eye point as a nalgebra Point3 for BSP traversal.
    pub fn eye_point(&self) -> Point3<f32> {
        let pos = self.position();
        Point3::new(pos.x, pos.y, pos.z)
    }
}
