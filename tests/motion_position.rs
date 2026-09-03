//! `Motion.current_pos` — the point `path_step` computes before it is rounded
//! into `current_coord`. The contract is one invariant, held at every step of
//! every path: `round_half_even(current_pos) == current_coord` on both axes.
//! Nothing a terminal shows depends on `current_pos`; it exists for a renderer
//! that can place a character between two cells.

use ttfx::engine::character::CharId;
use ttfx::engine::ctx::{Clock, EngineCtx, NoopHooks};
use ttfx::engine::terminal::{CharacterFilter, CharacterSort, TerminalConfig};
use ttfx::utils::easing::Easing;
use ttfx::utils::geometry::Coord;
use ttfx::utils::pycompat::round_half_even;
use ttfx::utils::rng::Rng;

fn make_ctx() -> EngineCtx {
    let config = TerminalConfig {
        canvas_width: 20,
        canvas_height: 10,
        ignore_terminal_dimensions: true,
        frame_rate: 0,
        ..Default::default()
    };
    EngineCtx::new("abcdef\nghijkl", config, Rng::seeded(0), Clock::virtual_with_frame_rate(60)).unwrap()
}

fn first_char(ctx: &mut EngineCtx) -> CharId {
    let mut rng = Rng::seeded(0);
    ctx.terminal.get_characters(&mut rng, CharacterFilter::default(), CharacterSort::TopToBottomLeftToRight)[0]
}

/// Tick `id` until its path completes (bounded), asserting the invariant at
/// every step. Returns how many steps strictly differed from the rounded
/// coordinate on BOTH axes — the count that says the float carries what the
/// integer drops.
fn drive(ctx: &mut EngineCtx, id: CharId) -> usize {
    let mut fractional_steps = 0;
    for _ in 0..500 {
        ctx.tick(&mut NoopHooks, id);
        let m = &ctx.terminal.arena[id.0 as usize].motion;
        let (col, row) = m.current_pos;
        assert_eq!(
            Coord::new(round_half_even(col), round_half_even(row)),
            m.current_coord,
            "current_pos {:?} does not round to current_coord",
            m.current_pos
        );
        let (sx, sy) = m.sub_cell();
        assert!(sx.abs() <= 0.5 && sy.abs() <= 0.5, "sub-cell remainder out of range: {:?}", (sx, sy));
        if sx != 0.0 && sy != 0.0 {
            fractional_steps += 1;
        }
        if m.active_path.is_none() {
            break;
        }
    }
    assert!(ctx.terminal.arena[id.0 as usize].motion.active_path.is_none(), "path never completed");
    fractional_steps
}

#[test]
fn new_motion_starts_exactly_on_its_cell() {
    let mut ctx = make_ctx();
    let id = first_char(&mut ctx);
    let m = &ctx.terminal.arena[id.0 as usize].motion;
    assert_eq!(m.current_pos, (m.current_coord.column as f64, m.current_coord.row as f64));
    assert_eq!(m.sub_cell(), (0.0, 0.0));
}

#[test]
fn a_diagonal_line_rounds_to_current_coord_at_every_step_and_differs_between_cells() {
    let mut ctx = make_ctx();
    let id = first_char(&mut ctx);
    {
        let motion = &mut ctx.terminal.arena[id.0 as usize].motion;
        motion.new_path(0.7, None, None, 0, false, "diag").unwrap();
        motion.paths.get_mut("diag").unwrap().new_waypoint(Coord::new(15, 8), None, "").unwrap();
    }
    ctx.activate_path(&mut NoopHooks, id, "diag");
    let fractional = drive(&mut ctx, id);
    assert!(fractional >= 10, "a diagonal at 0.7 cells/step must spend most steps between cells; got {fractional}");
    // Arrival is exact: the last step lands on the waypoint with no remainder.
    let m = &ctx.terminal.arena[id.0 as usize].motion;
    assert_eq!(m.current_coord, Coord::new(15, 8));
    assert_eq!(m.sub_cell(), (0.0, 0.0));
}

#[test]
fn a_bezier_and_an_eased_overshoot_hold_the_same_invariant() {
    let mut ctx = make_ctx();
    let id = first_char(&mut ctx);
    {
        let motion = &mut ctx.terminal.arena[id.0 as usize].motion;
        motion.new_path(1.3, Some(Easing::OutBack), None, 0, false, "curve").unwrap();
        let p = motion.paths.get_mut("curve").unwrap();
        p.new_waypoint(Coord::new(15, 8), None, "").unwrap();
        p.new_waypoint(Coord::new(18, 2), Some(vec![Coord::new(1, 1)]), "").unwrap();
    }
    ctx.activate_path(&mut NoopHooks, id, "curve");
    let fractional = drive(&mut ctx, id);
    assert!(fractional >= 3, "got {fractional}");
}

#[test]
fn set_coordinate_is_a_placement_with_no_remainder() {
    let mut ctx = make_ctx();
    let id = first_char(&mut ctx);
    {
        let motion = &mut ctx.terminal.arena[id.0 as usize].motion;
        motion.new_path(0.7, None, None, 0, false, "diag").unwrap();
        motion.paths.get_mut("diag").unwrap().new_waypoint(Coord::new(15, 8), None, "").unwrap();
    }
    ctx.activate_path(&mut NoopHooks, id, "diag");
    ctx.tick(&mut NoopHooks, id);
    let m = &mut ctx.terminal.arena[id.0 as usize].motion;
    assert_ne!(m.sub_cell(), (0.0, 0.0), "the first step of a diagonal is between cells");
    m.set_coordinate(Coord::new(4, 4));
    assert_eq!(m.current_coord, Coord::new(4, 4));
    assert_eq!(m.current_pos, (4.0, 4.0));
    assert_eq!(m.sub_cell(), (0.0, 0.0));
}

#[test]
fn set_position_rounds_half_to_even_like_the_path_does() {
    let mut ctx = make_ctx();
    let id = first_char(&mut ctx);
    let m = &mut ctx.terminal.arena[id.0 as usize].motion;
    m.set_position((2.5, 3.5));
    assert_eq!(m.current_coord, Coord::new(2, 4), "banker's rounding: 2.5 -> 2, 3.5 -> 4");
    assert_eq!(m.sub_cell(), (0.5, -0.5));
    m.set_position((7.25, 1.75));
    assert_eq!(m.current_coord, Coord::new(7, 2));
    assert_eq!(m.sub_cell(), (0.25, -0.25));
}
