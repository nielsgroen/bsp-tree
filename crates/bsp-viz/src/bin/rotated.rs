use std::hash::{Hash, Hasher};

use bsp_tree::{BspTree, BspVisitor, Plane3D, Polygon};
use macroquad::models::{draw_mesh, Mesh, Vertex};
use macroquad::prelude::*;
use nalgebra::{Point3, Rotation3, Unit, Vector3};

const NUM_CUBES: usize = 10;
const WORLD_SIZE: f32 = 30.0;
const MIN_CUBE_SIZE: f32 = 3.0;
const MAX_CUBE_SIZE: f32 = 8.0;

/// Simple seeded random number generator (LCG).
struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_f32(&mut self) -> f32 {
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((self.state >> 33) as f32) / (u32::MAX as f32 / 2.0)
    }

    /// Returns a random f32 in range [min, max]
    fn range(&mut self, min: f32, max: f32) -> f32 {
        min + self.next_f32() * (max - min)
    }
}

/// Generates a deterministic color from a polygon's vertices using hashing.
fn polygon_color(polygon: &Polygon) -> Color {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    for v in polygon.vertices() {
        v.x.to_bits().hash(&mut hasher);
        v.y.to_bits().hash(&mut hasher);
        v.z.to_bits().hash(&mut hasher);
    }
    let hash = hasher.finish();

    let r = ((hash >> 16) & 0xFF) as u8;
    let g = ((hash >> 8) & 0xFF) as u8;
    let b = (hash & 0xFF) as u8;

    // Ensure colors aren't too dark
    let r = r.max(60);
    let g = g.max(60);
    let b = b.max(60);

    Color::from_rgba(r, g, b, 255)
}

/// Creates a coplanar quad by projecting the 4th vertex onto the plane.
/// This fixes floating-point precision issues from rotation.
fn make_coplanar_quad(p0: Point3<f32>, p1: Point3<f32>, p2: Point3<f32>, p3: Point3<f32>) -> Polygon {
    // Define plane from first 3 points
    let plane = Plane3D::from_three_points(p0, p1, p2);
    // Project 4th point onto the plane to guarantee coplanarity
    let p3_projected = plane.project_point(p3);
    Polygon::new(vec![p0, p1, p2, p3_projected])
}

/// Generates the 6 face polygons of a rotated cube.
fn generate_rotated_cube(
    center: Point3<f32>,
    size: f32,
    rotation: &Rotation3<f32>,
) -> Vec<Polygon> {
    let half = size / 2.0;

    // Unit cube corners (centered at origin)
    let unit_corners = [
        Vector3::new(-half, -half, -half), // 0: left-bottom-back
        Vector3::new(half, -half, -half),  // 1: right-bottom-back
        Vector3::new(half, half, -half),   // 2: right-top-back
        Vector3::new(-half, half, -half),  // 3: left-top-back
        Vector3::new(-half, -half, half),  // 4: left-bottom-front
        Vector3::new(half, -half, half),   // 5: right-bottom-front
        Vector3::new(half, half, half),    // 6: right-top-front
        Vector3::new(-half, half, half),   // 7: left-top-front
    ];

    // Apply rotation and translate to center
    let corners: Vec<Point3<f32>> = unit_corners
        .iter()
        .map(|v| center + rotation * v)
        .collect();

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
            make_coplanar_quad(
                corners[indices[0]],
                corners[indices[1]],
                corners[indices[2]],
                corners[indices[3]],
            )
        })
        .collect()
}

/// Generates random rotated cubes in the world space.
fn generate_random_rotated_cubes(seed: u64) -> Vec<Polygon> {
    let mut rng = Rng::new(seed);
    let mut polygons = Vec::with_capacity(NUM_CUBES * 6);

    for _ in 0..NUM_CUBES {
        // Random position
        let x = (rng.next_f32() - 0.5) * WORLD_SIZE;
        let y = (rng.next_f32() - 0.5) * WORLD_SIZE;
        let z = (rng.next_f32() - 0.5) * WORLD_SIZE;
        let center = Point3::new(x, y, z);

        // Random size
        let size = rng.range(MIN_CUBE_SIZE, MAX_CUBE_SIZE);

        // Random rotation axis (normalized)
        let axis_x = rng.next_f32() - 0.5;
        let axis_y = rng.next_f32() - 0.5;
        let axis_z = rng.next_f32() - 0.5;
        let axis = Vector3::new(axis_x, axis_y, axis_z);

        // Avoid degenerate axis
        let axis = if axis.norm() > 0.01 {
            Unit::new_normalize(axis)
        } else {
            Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0))
        };

        // Random rotation angle (0 to 2*PI)
        let angle = rng.next_f32() * std::f32::consts::TAU;

        let rotation = Rotation3::from_axis_angle(&axis, angle);

        polygons.extend(generate_rotated_cube(center, size, &rotation));
    }

    polygons
}

/// Visitor that renders polygons using macroquad's 3D drawing.
struct RenderVisitor;

impl BspVisitor for RenderVisitor {
    fn visit(&mut self, polygons: &[Polygon]) {
        for polygon in polygons {
            draw_polygon(polygon);
        }
    }
}

/// Draws a single polygon by triangulating it (fan triangulation) using a Mesh.
fn draw_polygon(polygon: &Polygon) {
    let verts = polygon.vertices();
    if verts.len() < 3 {
        return;
    }

    let color = polygon_color(polygon);

    let mesh_vertices: Vec<Vertex> = verts
        .iter()
        .map(|p| Vertex::new2(vec3(p.x, p.y, p.z), vec2(0.0, 0.0), color))
        .collect();

    // Fan triangulation
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

/// Simple orbit camera state.
struct OrbitCamera {
    distance: f32,
    yaw: f32,
    pitch: f32,
    target: Vec3,
}

impl OrbitCamera {
    fn new() -> Self {
        Self {
            distance: 50.0,
            yaw: 0.0,
            pitch: 0.3,
            target: vec3(0.0, 0.0, 0.0),
        }
    }

    fn update(&mut self) {
        if is_mouse_button_down(MouseButton::Left) {
            let delta = mouse_delta_position();
            self.yaw -= delta.x * 2.0;
            self.pitch -= delta.y * 2.0;
        }

        self.pitch = self.pitch.clamp(-1.5, 1.5);

        let scroll = mouse_wheel().1;
        self.distance -= scroll * 3.0;
        self.distance = self.distance.clamp(10.0, 150.0);

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

    fn position(&self) -> Vec3 {
        let x = self.distance * self.pitch.cos() * self.yaw.sin();
        let y = self.distance * self.pitch.sin();
        let z = self.distance * self.pitch.cos() * self.yaw.cos();
        self.target + vec3(x, y, z)
    }

    fn to_camera3d(&self) -> Camera3D {
        Camera3D {
            position: self.position(),
            up: vec3(0.0, 1.0, 0.0),
            target: self.target,
            ..Default::default()
        }
    }

    fn eye_point(&self) -> Point3<f32> {
        let pos = self.position();
        Point3::new(pos.x, pos.y, pos.z)
    }
}

#[macroquad::main("BSP Rotated Cubes")]
async fn main() {
    println!("Generating {} random rotated cubes...", NUM_CUBES);
    let polygons = generate_random_rotated_cubes(42);
    let polygon_count = polygons.len();
    println!("Created {} polygons", polygon_count);

    println!("Building BSP tree...");
    let tree = BspTree::from_polygons(polygons);
    println!(
        "BSP tree built: {} polygons, depth {}",
        tree.polygon_count(),
        tree.depth()
    );

    let mut camera = OrbitCamera::new();

    loop {
        camera.update();

        clear_background(Color::from_rgba(15, 15, 25, 255));
        set_camera(&camera.to_camera3d());

        // Render polygons in back-to-front order (painter's algorithm)
        let mut visitor = RenderVisitor;
        tree.traverse_back_to_front(camera.eye_point(), &mut visitor);

        // Draw coordinate axes
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(8.0, 0.0, 0.0), RED);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 8.0, 0.0), GREEN);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 0.0, 8.0), BLUE);

        set_default_camera();

        draw_text(
            &format!("BSP Rotated Cubes - {} polygons", tree.polygon_count()),
            10.0,
            25.0,
            20.0,
            WHITE,
        );
        draw_text(
            &format!("Tree depth: {} | Original: {}", tree.depth(), polygon_count),
            10.0,
            45.0,
            18.0,
            GRAY,
        );
        draw_text("Drag mouse to rotate, scroll to zoom", 10.0, 70.0, 16.0, DARKGRAY);
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 90.0, 16.0, DARKGRAY);

        next_frame().await
    }
}
