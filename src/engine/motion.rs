//! Waypoint, Segment, Path, Motion — state from engine/motion.py. The stepping
//! logic that fires events (Path.step, Motion.move, activate_path) lives on
//! EngineCtx (ctx.rs) so actions run inline at upstream emission points.

use std::rc::Rc;

use crate::engine::events::WaypointKey;
use crate::utils::easing::Easing;
use crate::utils::geometry::{self, Coord};
use crate::utils::ordered_map::OrderedMap;
use crate::utils::pycompat::round_half_even;

/// Waypoints are cloned constantly — into segments, into origin segments on
/// every path activation, and into event keys — so both owned fields are
/// reference counted and a clone is two refcount bumps.
#[derive(Debug, Clone, PartialEq)]
pub struct Waypoint {
    pub waypoint_id: Rc<str>,
    pub coord: Coord,
    pub bezier_control: Option<Rc<[Coord]>>,
}

impl Waypoint {
    pub fn key(&self) -> WaypointKey {
        WaypointKey {
            coord: self.coord,
            waypoint_id: self.waypoint_id.clone(),
            bezier_control: self.bezier_control.clone(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Segment {
    pub start: Waypoint,
    pub end: Waypoint,
    pub distance: f64,
    pub enter_event_triggered: bool,
    pub exit_event_triggered: bool,
}

impl Segment {
    pub fn new(start: Waypoint, end: Waypoint, distance: f64) -> Self {
        Segment { start, end, distance, enter_event_triggered: false, exit_event_triggered: false }
    }
}

#[derive(Debug, Clone)]
pub struct Path {
    pub path_id: String,
    pub speed: f64,
    pub ease: Option<Easing>,
    pub layer: Option<i64>,
    pub hold_time: i64,
    pub loop_: bool,
    pub segments: Vec<Segment>,
    pub waypoints: Vec<Waypoint>,
    pub total_distance: f64,
    pub current_step: i64,
    pub max_steps: i64,
    pub hold_time_remaining: i64,
    pub last_distance_reached: f64,
    /// Distance of the synthetic origin segment set at activation (upstream
    /// keeps the Segment object; only its distance is read back).
    pub origin_segment: Option<Segment>,
}

impl Path {
    pub fn new(
        path_id: &str,
        speed: f64,
        ease: Option<Easing>,
        layer: Option<i64>,
        hold_time: i64,
        loop_: bool,
    ) -> Result<Self, String> {
        if speed <= 0.0 {
            return Err(format!("Path speed must be greater than 0. Received: {speed}"));
        }
        Ok(Path {
            path_id: path_id.to_string(),
            speed,
            ease,
            layer,
            hold_time,
            loop_,
            segments: Vec::new(),
            waypoints: Vec::new(),
            total_distance: 0.0,
            current_step: 0,
            max_steps: 0,
            hold_time_remaining: hold_time,
            last_distance_reached: 0.0,
            origin_segment: None,
        })
    }

    /// Path.new_waypoint: auto-id like scenes; duplicate explicit id errors.
    pub fn new_waypoint(
        &mut self,
        coord: Coord,
        bezier_control: Option<Vec<Coord>>,
        waypoint_id: &str,
    ) -> Result<Waypoint, String> {
        let waypoint_id: Rc<str> = if waypoint_id.is_empty() {
            let mut current_id = self.waypoints.len();
            loop {
                let candidate = current_id.to_string();
                if !self.waypoints.iter().any(|w| *w.waypoint_id == *candidate) {
                    break Rc::from(candidate);
                }
                current_id += 1;
            }
        } else {
            if self.waypoints.iter().any(|w| *w.waypoint_id == *waypoint_id) {
                return Err(format!("duplicate waypoint id: {waypoint_id}"));
            }
            Rc::from(waypoint_id)
        };
        // Python: empty tuple bezier_control is falsy -> None
        let bezier_control = bezier_control.filter(|v| !v.is_empty()).map(Rc::from);
        let waypoint = Waypoint { waypoint_id, coord, bezier_control };
        self.add_waypoint_to_path(waypoint.clone());
        Ok(waypoint)
    }

    /// Path._add_waypoint_to_path.
    fn add_waypoint_to_path(&mut self, waypoint: Waypoint) {
        self.waypoints.push(waypoint);
        if self.waypoints.len() < 2 {
            return;
        }
        let prev = &self.waypoints[self.waypoints.len() - 2];
        let waypoint = &self.waypoints[self.waypoints.len() - 1];
        let distance_from_previous = match &waypoint.bezier_control {
            Some(control) => geometry::find_length_of_bezier_curve(prev.coord, control, waypoint.coord),
            None => geometry::find_length_of_line(prev.coord, waypoint.coord, true),
        };
        self.total_distance += distance_from_previous;
        self.segments.push(Segment::new(prev.clone(), waypoint.clone(), distance_from_previous));
        self.max_steps = round_half_even(self.total_distance / self.speed);
    }

    pub fn query_waypoint(&self, waypoint_id: &str) -> Result<&Waypoint, String> {
        self.waypoints
            .iter()
            .find(|w| *w.waypoint_id == *waypoint_id)
            .ok_or_else(|| format!("waypoint not found: {waypoint_id}"))
    }
}

/// engine/motion.py Motion: per-character movement state. `active_path` and
/// `completed_path` are path ids (upstream holds object references; Path
/// equality is by id).
#[derive(Debug, Clone)]
pub struct Motion {
    pub paths: OrderedMap<Path>,
    /// Where the character is, in cells. Every consumer upstream reads this
    /// and nothing here changes how it is computed. Write it through
    /// `set_coordinate` / `set_position` so `current_pos` stays in step.
    pub current_coord: Coord,
    pub previous_coord: Coord,
    /// `current_coord` before rounding: `(column, row)` as f64 in the same
    /// 1-based bottom-left frame. Along a path this is the exact point
    /// `path_step` computed (`current_coord` is its banker's rounding, so the
    /// two differ by at most half a cell on each axis); after a
    /// `set_coordinate` it is the integer coordinate's exact value. A terminal
    /// cannot use it — the cell is the atom there — but a renderer that draws
    /// each character as a tile under a camera can, and without it every path
    /// reads as stepping. Additive: no frame ttfx emits depends on it.
    pub current_pos: (f64, f64),
    pub active_path: Option<Rc<str>>,
    pub completed_path: Option<Rc<str>>,
}

impl Motion {
    pub fn new(input_coord: Coord) -> Self {
        Motion {
            paths: OrderedMap::new(),
            current_coord: input_coord,
            previous_coord: Coord::new(-1, -1),
            current_pos: (input_coord.column as f64, input_coord.row as f64),
            active_path: None,
            completed_path: None,
        }
    }

    /// Motion.set_coordinate: place the character on a cell. The pre-rounded
    /// position becomes that cell exactly — a placement has no remainder.
    pub fn set_coordinate(&mut self, coord: Coord) {
        self.current_coord = coord;
        self.current_pos = (coord.column as f64, coord.row as f64);
    }

    /// Place the character at a pre-rounded point: `current_pos` takes it as
    /// given and `current_coord` takes its banker's rounding, the way
    /// `find_coord_on_line` / `find_coord_on_bezier_curve` round. This is the
    /// only way the two fields are ever set from a float, so
    /// `round_half_even(current_pos) == current_coord` holds by construction.
    pub fn set_position(&mut self, pos: (f64, f64)) {
        self.current_coord = Coord::new(round_half_even(pos.0), round_half_even(pos.1));
        self.current_pos = pos;
    }

    /// `current_pos - current_coord`: the sub-cell remainder in cells, each
    /// axis in `-0.5..=0.5` (exactly `±0.5` only at a banker's-rounding tie).
    /// Zero after any `set_coordinate`.
    pub fn sub_cell(&self) -> (f64, f64) {
        (
            self.current_pos.0 - self.current_coord.column as f64,
            self.current_pos.1 - self.current_coord.row as f64,
        )
    }

    /// Motion.new_path: auto-id probing; duplicate explicit id errors.
    pub fn new_path(
        &mut self,
        speed: f64,
        ease: Option<Easing>,
        layer: Option<i64>,
        hold_time: i64,
        loop_: bool,
        path_id: &str,
    ) -> Result<String, String> {
        let path_id = if path_id.is_empty() {
            let mut current_id = self.paths.len();
            loop {
                let candidate = current_id.to_string();
                if !self.paths.contains_key(&candidate) {
                    break candidate;
                }
                current_id += 1;
            }
        } else {
            if self.paths.contains_key(path_id) {
                return Err(format!("duplicate path id: {path_id}"));
            }
            path_id.to_string()
        };
        let path = Path::new(&path_id, speed, ease, layer, hold_time, loop_)?;
        self.paths.insert(path_id.clone(), path);
        Ok(path_id)
    }

    pub fn movement_is_complete(&self) -> bool {
        self.active_path.is_none()
    }

    /// Motion.deactivate_path: None clears unconditionally; otherwise only
    /// clears when the given path is the active one.
    pub fn deactivate_path(&mut self, path_id: Option<&str>) {
        match path_id {
            None => self.active_path = None,
            Some(id) => {
                if self.active_path.as_deref() == Some(id) {
                    self.active_path = None;
                }
            }
        }
    }
}
