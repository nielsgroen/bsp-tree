use bsp_tree::{BspTree, Polygon};
use bsp_viz::{generate_rotated_cube, OrbitCamera, TreeNavigator};
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

    fn range(&mut self, min: f32, max: f32) -> f32 {
        min + self.next_f32() * (max - min)
    }
}

/// Generates random rotated cubes in the world space.
fn generate_random_rotated_cubes(seed: u64) -> Vec<Polygon> {
    let mut rng = Rng::new(seed);
    let mut polygons = Vec::with_capacity(NUM_CUBES * 6);

    for _ in 0..NUM_CUBES {
        let x = (rng.next_f32() - 0.5) * WORLD_SIZE;
        let y = (rng.next_f32() - 0.5) * WORLD_SIZE;
        let z = (rng.next_f32() - 0.5) * WORLD_SIZE;
        let center = Point3::new(x, y, z);

        let size = rng.range(MIN_CUBE_SIZE, MAX_CUBE_SIZE);

        let axis_x = rng.next_f32() - 0.5;
        let axis_y = rng.next_f32() - 0.5;
        let axis_z = rng.next_f32() - 0.5;
        let axis = Vector3::new(axis_x, axis_y, axis_z);

        let axis = if axis.norm() > 0.01 {
            Unit::new_normalize(axis)
        } else {
            Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0))
        };

        let angle = rng.next_f32() * std::f32::consts::TAU;
        let rotation = Rotation3::from_axis_angle(&axis, angle);

        polygons.extend(generate_rotated_cube(center, size, &rotation));
    }

    polygons
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

    let mut camera = OrbitCamera::new(50.0, 0.0, 0.3).with_zoom(3.0, 10.0, 150.0);
    let mut navigator = TreeNavigator::new();

    loop {
        camera.update();
        navigator.update(&tree);

        clear_background(Color::from_rgba(15, 15, 25, 255));
        set_camera(&camera.to_camera3d());

        navigator.render(&tree, camera.eye_point());

        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(8.0, 0.0, 0.0), RED);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 8.0, 0.0), GREEN);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 0.0, 8.0), BLUE);

        set_default_camera();

        draw_text(
            &format!("BSP Rotated Cubes - Total: {} polygons", tree.polygon_count()),
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

        navigator.draw_ui(&tree, 70.0);

        draw_text("Drag mouse to rotate, scroll to zoom", 10.0, 155.0, 16.0, DARKGRAY);
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 175.0, 16.0, DARKGRAY);

        next_frame().await
    }
}
