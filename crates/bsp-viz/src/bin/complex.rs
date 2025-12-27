use bsp_tree::{BspTree, Polygon, Rectangle};
use bsp_viz::{generate_cube_polygons, generate_rotated_cube, OrbitCamera, TreeNavigator};
use macroquad::prelude::*;
use nalgebra::{Point3, Rotation3, Unit, Vector3};

/// Generates the complex scene with two cubes and a floor polygon.
fn generate_complex_scene() -> Vec<Polygon> {
    let mut polygons = Vec::new();

    // First cube: rotated around all three axes
    let rot_x = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::x()), 0.3);
    let rot_y = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::y()), 0.4);
    let rot_z = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::z()), 0.25);
    let rotation = rot_z * rot_y * rot_x;
    polygons.extend(generate_rotated_cube(Point3::new(-1.0, 0.0, 0.0), 0.8, &rotation));

    // Second cube: axis-aligned
    polygons.extend(generate_cube_polygons(Point3::new(1.0, 0.0, 0.0), 0.8));

    // Floor polygon at y = -1
    let floor = Rectangle::from_corners(
        Point3::new(-1.5, -1.0, -1.5),
        Point3::new(1.5, -1.0, -1.5),
        Point3::new(1.5, -1.0, 1.5),
        Point3::new(-1.5, -1.0, 1.5),
    );
    polygons.push(floor.into());

    polygons
}

#[macroquad::main("BSP Complex Scene")]
async fn main() {
    println!("Generating complex scene...");
    let polygons = generate_complex_scene();
    let polygon_count = polygons.len();
    println!("Created {} polygons (2 cubes + 1 floor)", polygon_count);

    println!("Building BSP tree...");
    let tree = BspTree::from_polygons(polygons);
    println!(
        "BSP tree built: {} polygons, depth {}",
        tree.polygon_count(),
        tree.depth()
    );

    let mut camera = OrbitCamera::new(5.0, 0.4, 0.4).with_zoom(0.5, 2.0, 20.0);
    let mut navigator = TreeNavigator::new();

    loop {
        camera.update();
        navigator.update(&tree);

        clear_background(Color::from_rgba(20, 20, 30, 255));
        set_camera(&camera.to_camera3d());

        navigator.render(&tree, camera.eye_point());

        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), RED);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), GREEN);
        draw_line_3d(vec3(0.0, 0.0, 0.0), vec3(0.0, 0.0, 1.0), BLUE);

        set_default_camera();

        draw_text(
            &format!("BSP Complex Scene - Total: {} polygons", tree.polygon_count()),
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
