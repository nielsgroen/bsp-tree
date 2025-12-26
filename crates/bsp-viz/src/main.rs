use macroquad::prelude::*;

#[macroquad::main("BSP Visualization")]
async fn main() {
    loop {
        clear_background(BLACK);

        draw_text("BSP Tree Visualization", 20.0, 40.0, 30.0, WHITE);

        next_frame().await
    }
}
