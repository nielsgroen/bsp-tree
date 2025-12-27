//! BSP tree navigation utilities for interactive visualization.

use bsp_tree::{BspNode, BspTree, PlaneSide};
use macroquad::prelude::*;
use nalgebra::Point3;

use crate::draw_polygon;

/// Direction taken at each node in the navigation path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Front,
    Back,
}

/// Interactive BSP tree navigator for exploring tree structure.
pub struct TreeNavigator {
    path: Vec<Direction>,
}

impl Default for TreeNavigator {
    fn default() -> Self {
        Self::new()
    }
}

impl TreeNavigator {
    /// Creates a new navigator starting at the root.
    pub fn new() -> Self {
        Self { path: Vec::new() }
    }

    /// Returns the current navigation path.
    pub fn path(&self) -> &[Direction] {
        &self.path
    }

    /// Returns the current depth in the tree.
    pub fn depth(&self) -> usize {
        self.path.len()
    }

    /// Attempts to navigate to the front child. Returns true if successful.
    pub fn go_front(&mut self, tree: &BspTree) -> bool {
        if let Some(node) = self.current_node(tree) {
            if node.front().is_some() {
                self.path.push(Direction::Front);
                return true;
            }
        }
        false
    }

    /// Attempts to navigate to the back child. Returns true if successful.
    pub fn go_back(&mut self, tree: &BspTree) -> bool {
        if let Some(node) = self.current_node(tree) {
            if node.back().is_some() {
                self.path.push(Direction::Back);
                return true;
            }
        }
        false
    }

    /// Navigates to the parent node. Returns true if not already at root.
    pub fn go_parent(&mut self) -> bool {
        self.path.pop().is_some()
    }

    /// Returns to the root node.
    pub fn go_root(&mut self) {
        self.path.clear();
    }

    /// Handles keyboard input for navigation.
    /// Returns true if navigation state changed.
    pub fn update(&mut self, tree: &BspTree) -> bool {
        let mut changed = false;

        if is_key_pressed(KeyCode::F) {
            changed = self.go_front(tree);
        }
        if is_key_pressed(KeyCode::B) {
            changed = self.go_back(tree);
        }
        if is_key_pressed(KeyCode::P) {
            changed = self.go_parent();
        }
        if is_key_pressed(KeyCode::R) {
            if !self.path.is_empty() {
                self.go_root();
                changed = true;
            }
        }

        changed
    }

    /// Returns a reference to the current node, if the tree is non-empty.
    pub fn current_node<'a>(&self, tree: &'a BspTree) -> Option<&'a BspNode> {
        tree.root().and_then(|root| get_node_at_path(root, &self.path))
    }

    /// Renders only the polygons in the current subtree with proper depth ordering.
    pub fn render(&self, tree: &BspTree, eye: Point3<f32>) {
        if let Some(node) = self.current_node(tree) {
            render_node_back_to_front(node, eye);
        }
    }

    /// Draws the navigation UI overlay.
    pub fn draw_ui(&self, tree: &BspTree, y_offset: f32) {
        let (node_polygons, has_front, has_back, is_leaf) = if let Some(node) = self.current_node(tree) {
            (
                node.polygon_count(),
                node.front().is_some(),
                node.back().is_some(),
                node.is_leaf(),
            )
        } else {
            (0, false, false, true)
        };

        // Build path string
        let path_str = if self.path.is_empty() {
            "root".to_string()
        } else {
            self.path
                .iter()
                .map(|d| match d {
                    Direction::Front => "F",
                    Direction::Back => "B",
                })
                .collect::<Vec<_>>()
                .join(" -> ")
        };

        draw_text(
            &format!("Subtree: {} polygons", node_polygons),
            10.0,
            y_offset,
            18.0,
            WHITE,
        );
        draw_text(
            &format!("Path: {} (depth {})", path_str, self.path.len()),
            10.0,
            y_offset + 20.0,
            18.0,
            YELLOW,
        );
        draw_text(
            &format!(
                "Children: {}{}{}",
                if has_front { "[F]ront " } else { "" },
                if has_back { "[B]ack " } else { "" },
                if is_leaf { "(leaf)" } else { "" }
            ),
            10.0,
            y_offset + 40.0,
            18.0,
            if is_leaf { ORANGE } else { GREEN },
        );
        draw_text(
            "[P]arent | [R]oot",
            10.0,
            y_offset + 60.0,
            16.0,
            DARKGRAY,
        );
    }
}

/// Navigates to a node following the path, returns None if path is invalid.
fn get_node_at_path<'a>(root: &'a BspNode, path: &[Direction]) -> Option<&'a BspNode> {
    let mut current = root;
    for dir in path {
        current = match dir {
            Direction::Front => current.front()?,
            Direction::Back => current.back()?,
        };
    }
    Some(current)
}

/// Recursively renders a node's subtree with back-to-front ordering.
fn render_node_back_to_front(node: &BspNode, eye: Point3<f32>) {
    let side = node.plane().classify_point(eye);

    match side {
        PlaneSide::Front | PlaneSide::OnPlane => {
            // Eye is in front: render back, then coplanar, then front
            if let Some(back) = node.back() {
                render_node_back_to_front(back, eye);
            }
            for polygon in node.all_coplanar() {
                draw_polygon(polygon);
            }
            if let Some(front) = node.front() {
                render_node_back_to_front(front, eye);
            }
        }
        PlaneSide::Back => {
            // Eye is in back: render front, then coplanar, then back
            if let Some(front) = node.front() {
                render_node_back_to_front(front, eye);
            }
            for polygon in node.all_coplanar() {
                draw_polygon(polygon);
            }
            if let Some(back) = node.back() {
                render_node_back_to_front(back, eye);
            }
        }
    }
}
