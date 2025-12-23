use box2d_lite_rs::dynamics::{BodyDef, JointDef, World, WorldConfig};
use box2d_lite_rs::math::{Mat22, Vec2};
use macroquad::prelude::*;
use ::rand::Rng;
use ::rand::thread_rng;

#[derive(Copy, Clone, Debug)]
enum Demo {
    Demo1,
    Demo2,
    Demo3,
    Demo4,
    Demo5,
    Demo6,
    Demo7,
    Demo8,
    Demo9,
}

impl Demo {
    fn name(self) -> &'static str {
        match self {
            Demo::Demo1 => "Demo 1: A Single Box",
            Demo::Demo2 => "Demo 2: Simple Pendulum",
            Demo::Demo3 => "Demo 3: Varying Friction Coefficients",
            Demo::Demo4 => "Demo 4: Randomized Stacking",
            Demo::Demo5 => "Demo 5: Pyramid Stacking",
            Demo::Demo6 => "Demo 6: A Teeter",
            Demo::Demo7 => "Demo 7: A Suspension Bridge",
            Demo::Demo8 => "Demo 8: Dominos",
            Demo::Demo9 => "Demo 9: Multi-pendulum",
        }
    }

    fn from_key(key: KeyCode) -> Option<Demo> {
        Some(match key {
            KeyCode::Key1 => Demo::Demo1,
            KeyCode::Key2 => Demo::Demo2,
            KeyCode::Key3 => Demo::Demo3,
            KeyCode::Key4 => Demo::Demo4,
            KeyCode::Key5 => Demo::Demo5,
            KeyCode::Key6 => Demo::Demo6,
            KeyCode::Key7 => Demo::Demo7,
            KeyCode::Key8 => Demo::Demo8,
            KeyCode::Key9 => Demo::Demo9,
            _ => return None,
        })
    }
}

#[derive(Clone, Debug)]
struct Camera2DView {
    zoom: f32,
    pan: Vec2,
}

impl Default for Camera2DView {
    fn default() -> Self {
        // Match the C++ sample defaults roughly.
        Self {
            zoom: 10.0,
            pan: Vec2::new(0.0, 8.0),
        }
    }
}

impl Camera2DView {
    fn world_to_screen(&self, p: Vec2) -> Vec2 {
        // Logical camera: screen maps to world with Y up.
        // Center screen at (0,0) world coordinates + pan.
        let w = screen_width();
        let h = screen_height();

        let aspect = w / h.max(1.0);
        let half_w = if aspect >= 1.0 { self.zoom * aspect } else { self.zoom };
        let half_h = if aspect >= 1.0 { self.zoom } else { self.zoom / aspect };

        let cam_center = Vec2::new(self.pan.x, self.pan.y);
        let ndc_x = (p.x - cam_center.x) / half_w;
        let ndc_y = (p.y - cam_center.y) / half_h;

        // ndc [-1,1] to pixels; flip Y because screen Y goes down.
        Vec2::new((ndc_x * 0.5 + 0.5) * w, (1.0 - (ndc_y * 0.5 + 0.5)) * h)
    }

    fn screen_to_world(&self, p: Vec2) -> Vec2 {
        let w = screen_width();
        let h = screen_height().max(1.0);

        let aspect = w / h;
        let half_w = if aspect >= 1.0 { self.zoom * aspect } else { self.zoom };
        let half_h = if aspect >= 1.0 { self.zoom } else { self.zoom / aspect };

        let cam_center = Vec2::new(self.pan.x, self.pan.y);

        let ndc_x = (p.x / w) * 2.0 - 1.0;
        let ndc_y = -((p.y / h) * 2.0 - 1.0);

        Vec2::new(cam_center.x + ndc_x * half_w, cam_center.y + ndc_y * half_h)
    }
}

fn draw_body(view: &Camera2DView, position: Vec2, rotation: f32, half_extents: Vec2, color: Color) {
    let r = Mat22::from_angle(rotation);

    let v1 = position + r * Vec2::new(-half_extents.x, -half_extents.y);
    let v2 = position + r * Vec2::new(half_extents.x, -half_extents.y);
    let v3 = position + r * Vec2::new(half_extents.x, half_extents.y);
    let v4 = position + r * Vec2::new(-half_extents.x, half_extents.y);

    let p1 = view.world_to_screen(v1);
    let p2 = view.world_to_screen(v2);
    let p3 = view.world_to_screen(v3);
    let p4 = view.world_to_screen(v4);

    draw_line(p1.x, p1.y, p2.x, p2.y, 1.5, color);
    draw_line(p2.x, p2.y, p3.x, p3.y, 1.5, color);
    draw_line(p3.x, p3.y, p4.x, p4.y, 1.5, color);
    draw_line(p4.x, p4.y, p1.x, p1.y, 1.5, color);
}

fn draw_joint_support(view: &Camera2DView, x: Vec2, p: Vec2) {
    let a = view.world_to_screen(x);
    let b = view.world_to_screen(p);
    draw_line(a.x, a.y, b.x, b.y, 1.0, Color::new(0.5, 0.5, 0.8, 1.0));
}

fn add_body(world: &mut World, def: BodyDef) -> usize {
    let h = world.create_body(def);
    h.0
}

fn add_joint(world: &mut World, def: JointDef) -> usize {
    let h = world.create_joint(def);
    h.0
}

fn init_demo(world: &mut World, demo: Demo) {
    world.clear();

    world.gravity = Vec2::new(0.0, -10.0);
    world.iterations = 10;

    match demo {
        Demo::Demo1 => {
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    mass: None,
                    ..Default::default()
                },
            );

            add_body(
                world,
                BodyDef {
                    width: Vec2::new(1.0, 1.0),
                    position: Vec2::new(0.0, 4.0),
                    mass: Some(200.0),
                    ..Default::default()
                },
            );
        }
        Demo::Demo2 => {
            let b1 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    friction: 0.2,
                    mass: None,
                    ..Default::default()
                },
            );
            let b2 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(1.0, 1.0),
                    position: Vec2::new(9.0, 11.0),
                    friction: 0.2,
                    mass: Some(100.0),
                    ..Default::default()
                },
            );

            add_joint(world, JointDef::new(
                box2d_lite_rs::dynamics::BodyHandle(b1),
                box2d_lite_rs::dynamics::BodyHandle(b2),
                Vec2::new(0.0, 11.0),
            ));
        }
        Demo::Demo3 => {
            // Ground
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    mass: None,
                    ..Default::default()
                },
            );

            // Ramps / boundaries
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(13.0, 0.25),
                    position: Vec2::new(-2.0, 11.0),
                    rotation: -0.25,
                    mass: None,
                    ..Default::default()
                },
            );
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(0.25, 1.0),
                    position: Vec2::new(5.25, 9.5),
                    mass: None,
                    ..Default::default()
                },
            );
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(13.0, 0.25),
                    position: Vec2::new(2.0, 7.0),
                    rotation: 0.25,
                    mass: None,
                    ..Default::default()
                },
            );
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(0.25, 1.0),
                    position: Vec2::new(-5.25, 5.5),
                    mass: None,
                    ..Default::default()
                },
            );
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(13.0, 0.25),
                    position: Vec2::new(-2.0, 3.0),
                    rotation: -0.25,
                    mass: None,
                    ..Default::default()
                },
            );

            let frictions = [0.75, 0.5, 0.35, 0.1, 0.0];
            for (i, friction) in frictions.into_iter().enumerate() {
                add_body(
                    world,
                    BodyDef {
                        width: Vec2::new(0.5, 0.5),
                        position: Vec2::new(-7.5 + 2.0 * i as f32, 14.0),
                        friction,
                        mass: Some(25.0),
                        ..Default::default()
                    },
                );
            }
        }
        Demo::Demo4 => {
            // Ground
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    friction: 0.2,
                    mass: None,
                    ..Default::default()
                },
            );

            // Randomized stacking in C++; here use small deterministic offsets.
            for i in 0..10 {
                let x = (i as f32 * 0.017).sin() * 0.1;
                add_body(
                    world,
                    BodyDef {
                        width: Vec2::new(1.0, 1.0),
                        position: Vec2::new(x, 0.51 + 1.05 * i as f32),
                        friction: 0.2,
                        mass: Some(1.0),
                        ..Default::default()
                    },
                );
            }
        }
        Demo::Demo5 => {
            // Ground
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    friction: 0.2,
                    mass: None,
                    ..Default::default()
                },
            );

            let mut x = Vec2::new(-6.0, 0.75);
            for i in 0..12 {
                let mut y = x;
                for _ in i..12 {
                    add_body(
                        world,
                        BodyDef {
                            width: Vec2::new(1.0, 1.0),
                            position: y,
                            friction: 0.2,
                            mass: Some(10.0),
                            ..Default::default()
                        },
                    );
                    y += Vec2::new(1.125, 0.0);
                }
                x += Vec2::new(0.5625, 2.0);
            }
        }
        Demo::Demo6 => {
            let b1 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    mass: None,
                    ..Default::default()
                },
            );
            let b2 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(12.0, 0.25),
                    position: Vec2::new(0.0, 1.0),
                    mass: Some(100.0),
                    ..Default::default()
                },
            );
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(0.5, 0.5),
                    position: Vec2::new(-5.0, 2.0),
                    mass: Some(25.0),
                    ..Default::default()
                },
            );
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(0.5, 0.5),
                    position: Vec2::new(-5.5, 2.0),
                    mass: Some(25.0),
                    ..Default::default()
                },
            );
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(1.0, 1.0),
                    position: Vec2::new(5.5, 15.0),
                    mass: Some(100.0),
                    ..Default::default()
                },
            );

            add_joint(
                world,
                JointDef::new(
                    box2d_lite_rs::dynamics::BodyHandle(b1),
                    box2d_lite_rs::dynamics::BodyHandle(b2),
                    Vec2::new(0.0, 1.0),
                ),
            );
        }
        Demo::Demo7 => {
            // Ground
            add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    friction: 0.2,
                    mass: None,
                    ..Default::default()
                },
            );

            let num_planks = 15;
            let mass = 50.0;
            let mut planks: Vec<usize> = Vec::with_capacity(num_planks);
            for i in 0..num_planks {
                let idx = add_body(
                    world,
                    BodyDef {
                        width: Vec2::new(1.0, 0.25),
                        position: Vec2::new(-8.5 + 1.25 * i as f32, 5.0),
                        friction: 0.2,
                        mass: Some(mass),
                        ..Default::default()
                    },
                );
                planks.push(idx);
            }

            // Match the C++ sample indexing:
            // In C++ `bodies[0]` is the ground, and `bodies[1..=numPlanks]` are planks.
            // Joints are created for i=0..numPlanks (ground->p0, p0->p1, ... p13->p14)
            // plus one extra joint connecting last plank back to ground.
            let ground_handle = box2d_lite_rs::dynamics::BodyHandle(0);
            let mut chain: Vec<box2d_lite_rs::dynamics::BodyHandle> = Vec::with_capacity(num_planks + 1);
            chain.push(ground_handle);
            for idx in &planks {
                chain.push(box2d_lite_rs::dynamics::BodyHandle(*idx));
            }

            let frequency_hz = 2.0;
            let damping_ratio = 0.7;
            let omega = 2.0 * std::f32::consts::PI * frequency_hz;
            let d = 2.0 * mass * damping_ratio * omega;
            let k = mass * omega * omega;
            let time_step = 1.0 / 60.0;
            let softness = 1.0 / (d + time_step * k);
            let bias_factor = time_step * k / (d + time_step * k);

            for i in 0..num_planks {
                let mut jd = JointDef::new(
                    chain[i],
                    chain[i + 1],
                    Vec2::new(-9.125 + 1.25 * i as f32, 5.0),
                );
                jd.softness = softness;
                jd.bias_factor = bias_factor;
                add_joint(world, jd);
            }

            // Close: last plank back to ground
            let mut jd = JointDef::new(
                chain[num_planks],
                chain[0],
                Vec2::new(-9.125 + 1.25 * num_planks as f32, 5.0),
            );
            jd.softness = softness;
            jd.bias_factor = bias_factor;
            add_joint(world, jd);
        }
        Demo::Demo8 => {
            let b1 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    mass: None,
                    ..Default::default()
                },
            );

            add_body(
                world,
                BodyDef {
                    width: Vec2::new(12.0, 0.5),
                    position: Vec2::new(-1.5, 10.0),
                    mass: None,
                    ..Default::default()
                },
            );

            for i in 0..10 {
                add_body(
                    world,
                    BodyDef {
                        width: Vec2::new(0.2, 2.0),
                        position: Vec2::new(-6.0 + 1.0 * i as f32, 11.125),
                        friction: 0.1,
                        mass: Some(10.0),
                        ..Default::default()
                    },
                );
            }

            add_body(
                world,
                BodyDef {
                    width: Vec2::new(14.0, 0.5),
                    position: Vec2::new(1.0, 6.0),
                    rotation: 0.3,
                    mass: None,
                    ..Default::default()
                },
            );

            let b2 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(0.5, 3.0),
                    position: Vec2::new(-7.0, 4.0),
                    mass: None,
                    ..Default::default()
                },
            );
            let b3 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(12.0, 0.25),
                    position: Vec2::new(-0.9, 1.0),
                    mass: Some(20.0),
                    ..Default::default()
                },
            );

            add_joint(
                world,
                JointDef::new(
                    box2d_lite_rs::dynamics::BodyHandle(b1),
                    box2d_lite_rs::dynamics::BodyHandle(b3),
                    Vec2::new(-2.0, 1.0),
                ),
            );

            let b4 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(0.5, 0.5),
                    position: Vec2::new(-10.0, 15.0),
                    mass: Some(10.0),
                    ..Default::default()
                },
            );

            add_joint(
                world,
                JointDef::new(
                    box2d_lite_rs::dynamics::BodyHandle(b2),
                    box2d_lite_rs::dynamics::BodyHandle(b4),
                    Vec2::new(-7.0, 15.0),
                ),
            );

            let b5 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(2.0, 2.0),
                    position: Vec2::new(6.0, 2.5),
                    friction: 0.1,
                    mass: Some(20.0),
                    ..Default::default()
                },
            );
            add_joint(
                world,
                JointDef::new(
                    box2d_lite_rs::dynamics::BodyHandle(b1),
                    box2d_lite_rs::dynamics::BodyHandle(b5),
                    Vec2::new(6.0, 2.6),
                ),
            );
            let b6 = add_body(
                world,
                BodyDef {
                    width: Vec2::new(2.0, 0.2),
                    position: Vec2::new(6.0, 3.6),
                    mass: Some(10.0),
                    ..Default::default()
                },
            );
            add_joint(
                world,
                JointDef::new(
                    box2d_lite_rs::dynamics::BodyHandle(b5),
                    box2d_lite_rs::dynamics::BodyHandle(b6),
                    Vec2::new(7.0, 3.5),
                ),
            );
        }
        Demo::Demo9 => {
            // Ground
            let ground = add_body(
                world,
                BodyDef {
                    width: Vec2::new(100.0, 20.0),
                    position: Vec2::new(0.0, -10.0),
                    friction: 0.2,
                    mass: None,
                    ..Default::default()
                },
            );

            let mass = 10.0;
            let frequency_hz = 4.0;
            let damping_ratio = 0.7;
            let omega = 2.0 * std::f32::consts::PI * frequency_hz;
            let d = 2.0 * mass * damping_ratio * omega;
            let k = mass * omega * omega;
            let time_step = 1.0 / 60.0;
            let softness = 1.0 / (d + time_step * k);
            let bias_factor = time_step * k / (d + time_step * k);

            let y = 12.0;
            let mut prev = ground;
            for i in 0..15 {
                let x = Vec2::new(0.5 + i as f32, y);
                let bi = add_body(
                    world,
                    BodyDef {
                        width: Vec2::new(0.75, 0.25),
                        position: x,
                        friction: 0.2,
                        mass: Some(mass),
                        ..Default::default()
                    },
                );

                let mut jd = JointDef::new(
                    box2d_lite_rs::dynamics::BodyHandle(prev),
                    box2d_lite_rs::dynamics::BodyHandle(bi),
                    Vec2::new(i as f32, y),
                );
                jd.softness = softness;
                jd.bias_factor = bias_factor;
                add_joint(world, jd);

                prev = bi;
            }
        }
    }
}

fn launch_bomb(world: &mut World, bomb_index: &mut Option<usize>) {
    let idx = match bomb_index {
        Some(i) => *i,
        None => {
            let i = add_body(
                world,
                BodyDef {
                    width: Vec2::new(1.0, 1.0),
                    position: Vec2::new(1.0, 1.0),
                    friction: 0.2,
                    mass: Some(50.0),
                    ..Default::default()
                },
            );
            *bomb_index = Some(i);
            i
        }
    };

    let mut rng = thread_rng();
    let b = &mut world.bodies[idx];
    b.position = Vec2::new(rng.gen_range(-15.0..=15.0), 15.0);
    b.rotation = rng.gen_range(-1.5..=1.5);
    b.velocity = -1.5 * b.position;
    b.angular_velocity = rng.gen_range(-20.0..=20.0);
}

#[macroquad::main("box2d-lite-rs samples")]
async fn main() {
    let mut world = World::with_config(
        Vec2::new(0.0, -10.0),
        10,
        WorldConfig {
            accumulate_impulses: true,
            warm_starting: true,
            position_correction: true,
        },
    );

    let mut demo = Demo::Demo1;
    let mut bomb_index: Option<usize> = None;
    let mut view = Camera2DView::default();

    let mut paused = false;
    let dt = 1.0 / 60.0;

    let mut last_mouse: Option<Vec2> = None;

    init_demo(&mut world, demo);

    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }

        // Input: demo switching 1-9
        for key in [
            KeyCode::Key1,
            KeyCode::Key2,
            KeyCode::Key3,
            KeyCode::Key4,
            KeyCode::Key5,
            KeyCode::Key6,
            KeyCode::Key7,
            KeyCode::Key8,
            KeyCode::Key9,
        ] {
            if is_key_pressed(key) {
                demo = Demo::from_key(key).unwrap();
                bomb_index = None;
                init_demo(&mut world, demo);
            }
        }

        if is_key_pressed(KeyCode::Space) {
            launch_bomb(&mut world, &mut bomb_index);
        }

        if is_key_pressed(KeyCode::A) {
            world.config.accumulate_impulses = !world.config.accumulate_impulses;
        }
        if is_key_pressed(KeyCode::W) {
            world.config.warm_starting = !world.config.warm_starting;
        }
        if is_key_pressed(KeyCode::P) {
            world.config.position_correction = !world.config.position_correction;
        }

        if is_key_pressed(KeyCode::K) {
            paused = !paused;
        }

        if is_key_pressed(KeyCode::R) {
            view = Camera2DView::default();
        }

        // Camera controls
        if is_key_down(KeyCode::Left) {
            view.pan.x -= 0.15 * view.zoom;
        }
        if is_key_down(KeyCode::Right) {
            view.pan.x += 0.15 * view.zoom;
        }
        if is_key_down(KeyCode::Down) {
            view.pan.y -= 0.15 * view.zoom;
        }
        if is_key_down(KeyCode::Up) {
            view.pan.y += 0.15 * view.zoom;
        }
        let wheel = mouse_wheel().1;
        if wheel.abs() > 0.0 {
            view.zoom = (view.zoom * (1.0 - wheel * 0.1)).clamp(1.0, 100.0);
        }

        // Mouse drag pan (right mouse button)
        let mouse = mouse_position();
        let mouse_v = Vec2::new(mouse.0, mouse.1);
        if is_mouse_button_down(MouseButton::Right) {
            if let Some(prev) = last_mouse {
                let w_prev = view.screen_to_world(prev);
                let w_now = view.screen_to_world(mouse_v);
                let delta = w_prev - w_now;
                view.pan += delta;
            }
            last_mouse = Some(mouse_v);
        } else {
            last_mouse = None;
        }

        if !paused {
            world.step(dt);
        } else if is_key_pressed(KeyCode::N) {
            world.step(dt);
        }

        clear_background(BLACK);

        // Draw bodies
        for (i, b) in world.bodies.iter().enumerate() {
            let half = 0.5 * b.width;
            let color = if Some(i) == bomb_index {
                Color::new(0.4, 0.9, 0.4, 1.0)
            } else {
                Color::new(0.8, 0.8, 0.9, 1.0)
            };
            draw_body(&view, b.position, b.rotation, half, color);
        }

        // Draw joints
        for j in &world.joints {
            // Match the C++ visualization: draw body center -> anchor for each body.
            let (x1, p1, x2, p2) = j.body_centers_and_anchors(&world);
            draw_joint_support(&view, x1, p1);
            draw_joint_support(&view, x2, p2);
        }

        // Draw contact points
        for arb in world.arbiters.values() {
            for c in &arb.contacts[..arb.num_contacts] {
                let p = view.world_to_screen(c.position);
                draw_circle(p.x, p.y, 3.0, RED);
            }
        }

        // UI overlay
        let overlay = format!(
            "{}\nKeys: 1-9 demos | Space bomb | A accum | P posCorr | W warm | K pause | N step | R reset view\nArrows pan | Wheel zoom | RMB drag pan\nzoom={:.2} pan=({:.2},{:.2})\naccum={} posCorr={} warm={} bodies={} joints={}",
            demo.name(),
            view.zoom,
            view.pan.x,
            view.pan.y,
            world.config.accumulate_impulses,
            world.config.position_correction,
            world.config.warm_starting,
            world.bodies.len(),
            world.joints.len()
        );
        draw_text(&overlay, 12.0, 20.0, 18.0, WHITE);

        next_frame().await;
    }
}
