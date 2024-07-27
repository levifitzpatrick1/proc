use comfy::*;

// makes the game
comfy_game!("Virtual Pets", VirtualPet, config);

pub enum Pets {
    FISH = 0,
    SNAKE = 1,
    LIZARD = 2,
}

// creates each config for the pets. these are the tuners for
// how they look and act
impl Pets {
    fn get_spine_config(&self) -> SpineConfig {
        match self {
            Pets::FISH => SpineConfig {
                joint_count: 10,
                body_widths: vec![0.68, 0.81, 0.84, 0.83, 0.77, 0.64, 0.51, 0.38, 0.32, 0.19],
                base_flexibility: 0.2,
                flexibility_growth: 0.05,
                joint_angle_constraint: PI / 8.0,
            },
            Pets::SNAKE => SpineConfig {
                joint_count: 48,
                body_widths: (0..48)
                    .map(|i| match i {
                        0 => 0.76,
                        1 => 0.60,
                        _ => 0.63 - i as f32 * 0.01,
                    })
                    .collect(),
                base_flexibility: 0.3,
                flexibility_growth: 0.01,
                joint_angle_constraint: PI / 32.0,
            },
            Pets::LIZARD => SpineConfig {
                joint_count: 14,
                body_widths: vec![
                    0.52, 0.58, 0.40, 0.60, 0.68, 0.71, 0.65, 0.50, 0.28, 0.15, 0.11, 0.09, 0.07,
                    0.07,
                ],
                base_flexibility: 0.2,
                flexibility_growth: 0.05,
                joint_angle_constraint: PI / 8.0,
            },
        }
    }
}
struct SpineConfig {
    joint_count: u8,
    body_widths: Vec<f32>,
    base_flexibility: f32,
    flexibility_growth: f32,
    joint_angle_constraint: f32,
}
pub struct VirtualPet {
    _pet_name: String,
    _pet_type: Pets,
    spine: Spine,
    arms: Option<Vec<Spine>>,
    arm_desired: Option<Vec<Vec2>>,
    tail: Spine,
    time: f32,

    position: Vec2,
    velocity: Vec2,
    target: Vec2,
    max_speed: f32,
    max_force: f32,
    world_bounds: Vec2,
    target_change_timer: f32,
    initialized: bool,
}

// wanted full screen
fn config(config: GameConfig) -> GameConfig {
    GameConfig {
        fullscreen: true,
        ..config
    }
}
impl GameLoop for VirtualPet {
    // on start need to make the pet
    fn new(_c: &mut EngineState) -> Self {
        let _pet_type = Pets::LIZARD; // <- change here for different Pets
        let config = _pet_type.get_spine_config();
        let spine = Spine::new(Vec2::new(0.0, 0.0), config);

        // make some legs that are just little spines for the lizard
        let (arms, arm_desired) = if let Pets::LIZARD = _pet_type {
            let mut arms = Vec::new();
            let mut arm_desired = Vec::new();
            for _i in 0..4 {
                let arm_config = SpineConfig {
                    joint_count: 3,
                    body_widths: vec![0.4, 0.3, 0.2],
                    base_flexibility: 0.5,
                    flexibility_growth: 0.1,
                    joint_angle_constraint: PI / 4.0,
                };
                arms.push(Spine::new(Vec2::ZERO, arm_config));
                arm_desired.push(Vec2::ZERO);
            }
            (Some(arms), Some(arm_desired))
        } else {
            (None, None)
        };

        let tail_config = SpineConfig {
            joint_count: 3,
            body_widths: vec![0.3, 0.2, 0.1],
            base_flexibility: 0.5,
            flexibility_growth: 0.1,
            joint_angle_constraint: PI / 4.0,
        };
        let tail: Spine = Spine::new(Vec2::ZERO, tail_config);

        Self {
            _pet_name: String::from("pet"),
            _pet_type,
            spine,
            arms,
            arm_desired,
            tail,
            time: 0.0,

            position: Vec2::ZERO,
            velocity: Vec2::ZERO,
            target: Vec2::ZERO,
            max_speed: 1.0,
            max_force: 2.0,
            world_bounds: Vec2::new(2.0, 2.0),
            target_change_timer: 0.0,
            initialized: false,
        }
    }

    fn update(&mut self, _c: &mut EngineContext) {
        // need to load the game before it is able to read
        // the screen size. was getting crashes when
        // trying to call the screen_width() ect
        if !self.initialized {
            self.initialize();
        }

        let delta_time = delta();
        self.time += delta_time;

        self.target_change_timer -= delta_time;
        if self.target_change_timer <= 0.0 {
            self.set_new_target();
        }

        // finds the normal vector that points in the direction
        // of the target in respect to the current position.
        // then scales it by the max speed.
        let desired = (self.target - self.position).normalize() * self.max_speed;
        // we want to clamp the acceleration, so we find
        // our change in velocity then clamp that vector length
        // to the 'max_force' or max acceleration of the system
        let mut steering = desired - self.velocity;
        steering = steering.clamp_length_max(self.max_force);

        // steering is our acceleration vector, so
        // multiplying by time gives us our current velocity.
        // we need to clamp that as well, to our max_speed
        self.velocity += steering * delta_time;
        self.velocity = self.velocity.clamp_length_max(self.max_speed);

        // get our current position from v * t
        self.position += self.velocity * delta_time;

        // make sure the x and y positions are inside
        // the bounds of our world.
        //
        // TODO: this needs to get changed to move the body as well
        // when an animal reaches an edge, the head snaps but the body
        // floats over.
        self.position.x = (self.position.x + self.world_bounds.x / 2.0) % self.world_bounds.x
            - self.world_bounds.x / 2.0;
        self.position.y = (self.position.y + self.world_bounds.y / 2.0) % self.world_bounds.y
            - self.world_bounds.y / 2.0;

        // hover -> VVVVV to see what update does
        self.spine.update(self.position);

        let last_segment: usize = self.spine.joints.len();
        let start_pos = self.spine.get_pos(last_segment, 0.0, 0.0);
        self.tail.joints[2] = start_pos;
        self.tail.draw();

        // if the animal has arms, we need to animate those as well
        if let (Some(arms), Some(arm_desired)) = (&mut self.arms, &mut self.arm_desired) {
            for (i, arm) in arms.iter_mut().enumerate() {
                // even arms are on one side, odd on the other
                let side = if i % 2 == 0 { 1.0 } else { -1.0 };
                // arms attach on body segments 3 and 7
                let body_index = if i < 2 { 3 } else { 7 };
                // front arms are at a 45 deg angle
                // back arms are at a 60 deg angle
                let angle = if i < 2 { PI / 4.0 } else { PI / 3.0 };
                // get the foot position based off of the
                // current body section
                let desired_pos = self.spine.get_pos(body_index, angle * side, 0.8);

                // makes it so that the feet only move when
                // the change is large enough
                if (desired_pos - arm_desired[i]).length() > 2.0 {
                    arm_desired[i] = desired_pos;
                }

                //anchors the arm to the body
                let start_pos = self.spine.get_pos(body_index, PI / 2.0 * side, -0.2);
                // creates a target point 60% of the way to the desired point
                // to smooth the motion
                arm.update(arm_desired[i].lerp(arm.joints[0], 0.6));
                arm.joints[2] = start_pos;
            }
        }

        self.draw();
    }
}
impl VirtualPet {
    fn initialize(&mut self) {
        /*!
            sets up the world bounds and
            max speed / force for animal
        */
        let aspect_ractio = aspect_ratio();

        let world_height = 20.0;

        self.world_bounds = Vec2::new(world_height * aspect_ractio, world_height);
        self.position = Vec2::ZERO;
        self.set_new_target();

        self.max_speed = world_height / 5.0;
        self.max_force = world_height / 2.0;

        self.initialized = true;
    }

    fn draw(&self) {
        self.spine.draw();

        if let Some(arms) = &self.arms {
            for arm in arms {
                arm.draw();
            }
        }

        // drawing eyes
        let eye_size = 0.24;
        let eye_offset = 0.18;
        draw_circle(
            self.spine.get_pos(0, 3.0 * PI / 5.0, -eye_offset),
            eye_size,
            WHITE,
            0,
        );
        draw_circle(
            self.spine.get_pos(0, -3.0 * PI / 5.0, -eye_offset),
            eye_size,
            WHITE,
            0,
        );
    }

    fn set_new_target(&mut self) {
        /*!
            picks a random point within the world
            and a random amount of time to switch
            targets.
        */
        let mut rng = rand::thread_rng();
        self.target = Vec2::new(
            rng.gen_range(-self.world_bounds.x / 2.0..self.world_bounds.x / 2.0),
            rng.gen_range(-self.world_bounds.y / 2.0..self.world_bounds.y / 2.0),
        );
        self.target_change_timer = rng.gen_range(3.0..6.0);
    }
}

struct Spine {
    joints: Vec<Vec2>,
    segment_lengths: Vec<f32>,
    joint_weights: Vec<f32>,
    flexibility: Vec<f32>,
    joint_angle_constraint: f32,
}

impl Spine {
    fn new(origin: Vec2, config: SpineConfig) -> Spine {
        /*!
            Creates a new Spine object using the config

            Params: origin as {Vec2} | Config as SpineConfig

            Returns: Spine Struct
        */
        let mut joints = Vec::with_capacity(config.joint_count as usize);
        let mut segment_lengths = Vec::with_capacity(config.joint_count as usize - 1);
        let mut flexibility = Vec::with_capacity(config.joint_count as usize);

        let mut current_pos = origin;

        for i in 0..config.joint_count {
            let progress = i as f32 / (config.joint_count - 1) as f32;

            joints.push(current_pos);

            if i > 0 {
                let segment_length = config.body_widths[i as usize - 1] * 2.0;
                segment_lengths.push(segment_length);
                current_pos += Vec2::new(segment_length, 0.0);
            }

            let flex = config.base_flexibility + progress * config.flexibility_growth;
            flexibility.push(flex);
        }

        Spine {
            joints,
            segment_lengths,
            joint_weights: config.body_widths,
            flexibility,
            joint_angle_constraint: config.joint_angle_constraint,
        }
    }

    fn update(&mut self, target: Vec2) {
        /*!
            - sets the head to the passed in Vec2
            - for each following joint:
                - get the direction by normalizing the distance between current and last joint
                - get the previous direction, if it exists, by calcing the last 2 joints
                - get the angle between the current direction and the last
                - clamp the angle to +/- the angle constraint
                - rotate the direction vector by the clampled angle
                - current pos is the previous joint, + the direction vector scaled by the segment length
                - use linear interpolation (lerp) with flexibility of joint
        */

        self.joints[0] = target;

        for i in 1..self.joints.len() {
            let dir = (self.joints[i] - self.joints[i - 1]).normalize();
            //let ideal_position = self.joints[i - 1] + dir * self.segment_lengths[i - 1];

            let prev_dir = if i > 1 {
                (self.joints[i - 1] - self.joints[i - 2]).normalize()
            } else {
                dir
            };
            let angle = dir.angle_between(prev_dir);

            let constrained_angle =
                angle.clamp(-self.joint_angle_constraint, self.joint_angle_constraint);

            let rotated_dir = rotate_vec2(dir, constrained_angle);
            let constrained_position =
                self.joints[i - 1] + rotated_dir * self.segment_lengths[i - 1];

            let flex_factor = self.flexibility[i];
            self.joints[i] = self.joints[i].lerp(constrained_position, flex_factor);
        }
    }

    fn get_pos(&self, i: usize, angle_offset: f32, length_offset: f32) -> Vec2 {
        /*!
            returns a 2d vector representing an offset
            point from a specified body segment
        */
        if i >= self.joints.len() - 1 {
            return self.joints[self.joints.len() - 1];
        }

        let joint = self.joints[i];
        let next_joint = self.joints[i + 1];
        let angle = (next_joint - joint).angle() + angle_offset;
        let length = self.joint_weights[i] + length_offset;

        Vec2::new(
            joint.x + angle.cos() * length,
            joint.y + angle.sin() * length,
        )
    }

    fn draw(&self) {
        /*!
            - Iterates over each joint in the spine
            - draws a circle for each of the joints, with the size being its weight
            - draws a line connecting each of the spine points

        */
        for (i, point) in self.joints.iter().enumerate() {
            let color = if i == 0 { RED } else { BLUE };
            draw_circle_outline(*point, self.joint_weights[i], 0.025, color, 0);
            if i > 0 {
                draw_line(
                    self.joints[i - 1],
                    *point,
                    self.joint_weights[i] * 0.2,
                    WHITE,
                    0,
                );
            }
        }
    }
}

fn rotate_vec2(v: Vec2, angle: f32) -> Vec2 {
    /*!
        Params: {Vec2} as the vector to rotate |
                {f32} as the angle to rotate around

        rotates a vecotor to the desired angle
    */
    let cos_angle = angle.cos();
    let sin_angle = angle.sin();
    Vec2::new(
        v.x * cos_angle - v.y * sin_angle,
        v.x * sin_angle + v.y * cos_angle,
    )
}
