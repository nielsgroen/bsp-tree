use bsp_tree::{BspTree, Polygon};
use bsp_viz::{generate_cube_polygons, OrbitCamera, TreeNavigator};
use macroquad::prelude::*;
use nalgebra::Point3;

const NUM_CUBES: usize = 100;
const WORLD_SIZE: f32 = 50.0;
const MIN_CUBE_SIZE: f32 = 1.0;
const MAX_CUBE_SIZE: f32 = 5.0;

/// Generates random cubes in the world space.
fn generate_random_cubes(seed: u64) -> Vec<Polygon> {
    let mut state = seed;
    let mut next_random = || -> f32 {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((state >> 33) as f32) / (u32::MAX as f32 / 2.0)
    };

    let mut polygons = Vec::with_capacity(NUM_CUBES * 6);

    for _ in 0..NUM_CUBES {
        let x = (next_random() - 0.5) * WORLD_SIZE;
        let y = (next_random() - 0.5) * WORLD_SIZE;
        let z = (next_random() - 0.5) * WORLD_SIZE;
        let size = MIN_CUBE_SIZE + next_random() * (MAX_CUBE_SIZE - MIN_CUBE_SIZE);

        let center = Point3::new(x, y, z);
        polygons.extend(generate_cube_polygons(center, size));
    }

    polygons
}

#[macroquad::main("BSP Visualization")]
async fn main() {
    println!("Generating {} random cubes...", NUM_CUBES);
    let polygons = generate_random_cubes(42);
    let polygon_count = polygons.len();
    println!("Created {} polygons", polygon_count);

    println!("Building BSP tree...");
    let tree = BspTree::from_polygons(polygons);
    println!(
        "BSP tree built: {} polygons, depth {}",
        tree.polygon_count(),
        tree.depth()
    );

    let mut camera = OrbitCamera::new(80.0, 0.0, 0.3);
    let mut navigator = TreeNavigator::new();

    loop {
        camera.update();
        navigator.update(&tree);

        clear_background(Color::from_rgba(20, 20, 30, 255));
        set_camera(&camera.to_camera3d());

        // Render current subtree with proper depth ordering
        navigator.render(&tree, camera.eye_point());

        // Draw coordinate axes
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(10.0, 0.0, 0.0), RED);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 10.0, 0.0), GREEN);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 0.0, 10.0), BLUE);

        set_default_camera();

        // Draw UI
        draw_text(
            &format!("BSP Tree Visualization - Total: {} polygons", tree.polygon_count()),
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

        // Navigator UI
        navigator.draw_ui(&tree, 70.0);

        draw_text("Drag mouse to rotate, scroll to zoom", 10.0, 155.0, 16.0, DARKGRAY);
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 175.0, 16.0, DARKGRAY);

        next_frame().await
    }
}
