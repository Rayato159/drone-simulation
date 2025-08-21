use std::f32::consts::PI;

use bevy::{input::mouse::MouseMotion, prelude::*, window::WindowMode};
use bevy_rapier3d::prelude::*;

const FOLLOW_DIST: f32 = 15.0;
const FOLLOW_PITCH: f32 = 10.0;
const SENSITIVITY: f32 = 0.005;
const GRAVITY: f32 = 9.81;

#[derive(Component)]
pub struct Drone;

#[derive(Component)]
pub struct DroneCamera;

#[derive(Component)]
pub struct HoverPid {
    pub kp: f32,
    pub min_kp: f32,
    pub max_kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub prev_e: f32,
    pub integral_e: f32,
    pub target_y: f32,
    pub v_rate: f32,
    pub min_y: f32,
    pub max_y: f32,
}

#[derive(Component)]
pub struct PitchPid {
    pub kp: f32,
    pub min_kp: f32,
    pub max_kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub prev_e: f32,
    pub integral_e: f32,
    pub target_angle: f32,
    pub angle_rate: f32,
    pub min_angle: f32,
    pub max_angle: f32,
}

#[derive(Component)]
pub struct RollPid {
    pub kp: f32,
    pub min_kp: f32,
    pub max_kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub prev_e: f32,
    pub integral_e: f32,
    pub target_angle: f32,
    pub angle_rate: f32,
    pub min_angle: f32,
    pub max_angle: f32,
}

#[derive(Component)]
pub struct YawPid {
    pub kp: f32,
    pub min_kp: f32,
    pub max_kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub prev_e: f32,
    pub integral_e: f32,
    pub target_angle: f32,
    pub angle_rate: f32,
    pub min_angle: f32,
    pub max_angle: f32,
}

#[derive(Component)]
pub struct OutputYText;

#[derive(Component)]
pub struct TargetYText;

#[derive(Component)]
pub struct OutputPitchText;

#[derive(Component)]
pub struct TargetPitchText;

#[derive(Component)]
pub struct OutputRollText;

#[derive(Component)]
pub struct TargetRollText;

#[derive(Component)]
pub struct OutputYawText;

#[derive(Component)]
pub struct TargetYawText;

#[derive(Component)]
pub struct EngineText;

#[derive(Component)]
pub struct EngineUI;

#[derive(States, Default, Debug, Clone, PartialEq, Eq, Hash)]
pub enum EngineState {
    On,
    #[default]
    Off,
}

#[derive(Resource)]
pub struct Delay {
    pub timer: Timer,
}

impl Delay {
    pub fn new(duration: f32) -> Self {
        Self {
            timer: Timer::from_seconds(duration, TimerMode::Repeating),
        }
    }
}

#[derive(Resource)]
pub struct DroneCameraParams {
    pub yaw: f32,
    pub pitch: f32,
    pub radius: f32,
    pub sensitivity: f32,
}

impl Default for DroneCameraParams {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: FOLLOW_PITCH.to_radians(),
            radius: FOLLOW_DIST,
            sensitivity: SENSITIVITY,
        }
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Drone Sim".into(),
                resizable: true,
                mode: WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                ..Default::default()
            }),
            ..Default::default()
        }))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .insert_resource(Delay::new(0.05))
        .insert_resource(DroneCameraParams::default())
        .init_state::<EngineState>()
        .add_systems(Startup, spawn_floor)
        .add_systems(Startup, spawn_drone)
        .add_systems(Startup, spawn_light)
        .add_systems(Startup, spawn_camera)
        .add_systems(Startup, spawn_ui)
        .add_systems(
            Update,
            (
                manual_control,
                control_camera_mouse,
                update_engine_ui,
                update_output_y_text,
                update_target_y_text,
                update_output_pitch_text,
                update_target_pitch_text,
                update_output_roll_text,
                update_target_roll_text,
                update_output_yaw_text,
                update_target_yaw_text,
                update_camera_pos,
            ),
        )
        .add_systems(
            Update,
            update_drone_forces.run_if(in_state(EngineState::On)),
        )
        .add_systems(OnExit(EngineState::On), engine_off)
        .run();
}

pub fn spawn_floor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(500., 0.1, 500.))),
            MeshMaterial3d(materials.add(Color::WHITE)),
            Transform::from_xyz(0.0, 0.0, 0.0),
        ))
        .insert(RigidBody::Fixed)
        .insert(Collider::cuboid(500. / 2., 0.1 / 2.0, 500. / 2.));
}

pub fn spawn_drone(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Drone,
            Mesh3d(meshes.add(Cuboid::new(1.0, 0.5, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
            Transform::from_xyz(0.0, 3.0, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(1.0 / 2.0, 0.5 / 2.0, 1.0 / 2.0),
            GravityScale(1.0),
            ExternalForce::default(),
            ColliderMassProperties::Density(1.0),
            ReadMassProperties::default(),
            Velocity::zero(),
        ))
        .insert(HoverPid {
            kp: 3.0,
            min_kp: 0.6,
            max_kp: 6.0,
            ki: 0.23,
            kd: 1.09,
            prev_e: 0.0,
            integral_e: 0.0,
            target_y: 0.0,
            v_rate: 2.0,
            min_y: 0.0,
            max_y: 120.0,
        })
        .insert(PitchPid {
            kp: 5.0,
            min_kp: 2.0,
            max_kp: 6.0,
            ki: 0.1,
            kd: 1.2,
            prev_e: 0.0,
            integral_e: 0.0,
            target_angle: 0.0 * PI / 180.0,
            angle_rate: 5.0 * PI / 180.0,
            min_angle: -30.0 * PI / 180.0,
            max_angle: 30.0 * PI / 180.0,
        })
        .insert(RollPid {
            kp: 5.0,
            min_kp: 2.0,
            max_kp: 6.0,
            ki: 0.1,
            kd: 1.2,
            prev_e: 0.0,
            integral_e: 0.0,
            target_angle: 0.0 * PI / 180.0,
            angle_rate: 5.0 * PI / 180.0,
            min_angle: -30.0 * PI / 180.0,
            max_angle: 30.0 * PI / 180.0,
        })
        .insert(YawPid {
            kp: 5.0,
            min_kp: 2.0,
            max_kp: 6.0,
            ki: 0.1,
            kd: 1.2,
            prev_e: 0.0,
            integral_e: 0.0,
            target_angle: 0.0 * PI / 180.0,
            angle_rate: 5.0 * PI / 180.0,
            min_angle: -PI * 2.0,
            max_angle: PI * 2.0,
        });
}

pub fn update_drone_forces(
    time: Res<Time>,
    mut drone_query: Query<
        (
            &Transform,
            &mut ExternalForce,
            &mut HoverPid,
            &mut PitchPid,
            &mut RollPid,
            &mut YawPid,
            &ReadMassProperties,
        ),
        With<Drone>,
    >,
) {
    let dt = time.delta_secs();

    for (tf, mut force, mut ctl_y, mut ctl_pitch, mut ctl_roll, mut ctl_yaw, mass_props) in
        drone_query.iter_mut()
    {
        // === Hover (Y) ===
        let y = tf.translation.y;
        let e_y = ctl_y.target_y - y;
        ctl_y.integral_e += e_y * dt;
        let norm_y = (y / ctl_y.max_y).clamp(0.0, 1.0);
        ctl_y.kp = ctl_y.min_kp + (ctl_y.max_kp - ctl_y.min_kp) * norm_y;
        let a_y =
            ctl_y.kp * e_y + ctl_y.ki * ctl_y.integral_e + ctl_y.kd * (e_y - ctl_y.prev_e) / dt;
        ctl_y.prev_e = e_y;
        let thrust_total = mass_props.mass * (a_y + GRAVITY);

        // === Orientation
        let (yaw, pitch, roll) = tf.rotation.to_euler(EulerRot::YXZ);

        let e_pitch = angle_error(ctl_pitch.target_angle, pitch);
        ctl_pitch.integral_e += e_pitch * dt;
        let alpha_pitch = ctl_pitch.kp * e_pitch
            + ctl_pitch.ki * ctl_pitch.integral_e
            + ctl_pitch.kd * (e_pitch - ctl_pitch.prev_e) / dt;
        ctl_pitch.prev_e = e_pitch;
        let torque_x = mass_props.principal_inertia.x * alpha_pitch;

        let e_roll = angle_error(ctl_roll.target_angle, roll);
        ctl_roll.integral_e += e_roll * dt;
        let alpha_roll = ctl_roll.kp * e_roll
            + ctl_roll.ki * ctl_roll.integral_e
            + ctl_roll.kd * (e_roll - ctl_roll.prev_e) / dt;
        ctl_roll.prev_e = e_roll;
        let torque_z = mass_props.principal_inertia.z * alpha_roll;

        let e_yaw = angle_error(ctl_yaw.target_angle, yaw);
        ctl_yaw.integral_e += e_yaw * dt;
        let alpha_yaw = ctl_yaw.kp * e_yaw
            + ctl_yaw.ki * ctl_yaw.integral_e
            + ctl_yaw.kd * (e_yaw - ctl_yaw.prev_e) / dt;
        ctl_yaw.prev_e = e_yaw;
        let torque_y = mass_props.principal_inertia.y * alpha_yaw;

        // === Apply Total Force + Torque ===
        force.force = tf.up() * thrust_total;
        force.torque = Vec3::new(torque_x, torque_y, torque_z);
    }
}

#[inline]
fn angle_error(target: f32, current: f32) -> f32 {
    let angle = target - current;

    let y = (angle + PI).rem_euclid(2.0 * PI) - PI;
    if y == -PI { PI } else { y }
}

pub fn manual_control(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut drone_query: Query<(&mut HoverPid, &mut PitchPid, &mut RollPid, &mut YawPid), With<Drone>>,
    mut next_engine_state: ResMut<NextState<EngineState>>,
    engine_state: Res<State<EngineState>>,
    mut delay: ResMut<Delay>,
    time: Res<Time>,
) {
    for (mut ctl_y, mut ctl_pitch, mut ctl_roll, mut ctl_yaw) in drone_query.iter_mut() {
        if keyboard.just_pressed(KeyCode::KeyP) {
            if *engine_state.get() == EngineState::On {
                next_engine_state.set(EngineState::Off);
            } else {
                next_engine_state.set(EngineState::On);
            }
        }

        if keyboard.pressed(KeyCode::Space) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_y.target_y += ctl_y.v_rate;
                    ctl_y.target_y = ctl_y.target_y.min(ctl_y.max_y); // Prevent exceeding a maximum height
                }
            }
        }
        if keyboard.pressed(KeyCode::ControlLeft) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_y.target_y -= ctl_y.v_rate;
                    ctl_y.target_y = ctl_y.target_y.max(ctl_y.min_y); // Prevent going below ground level
                }
            }
        }

        if keyboard.pressed(KeyCode::KeyW) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_pitch.target_angle -= ctl_pitch.angle_rate;
                    ctl_pitch.target_angle = ctl_pitch.target_angle.max(ctl_pitch.min_angle);
                }
            }
        }
        if keyboard.just_released(KeyCode::KeyW) {
            ctl_pitch.target_angle = 0.0;
        }

        if keyboard.pressed(KeyCode::KeyS) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_pitch.target_angle += ctl_pitch.angle_rate;
                    ctl_pitch.target_angle = ctl_pitch.target_angle.min(ctl_pitch.max_angle);
                }
            }
        }
        if keyboard.just_released(KeyCode::KeyS) {
            ctl_pitch.target_angle = 0.0;
        }

        if keyboard.pressed(KeyCode::KeyD) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_roll.target_angle -= ctl_roll.angle_rate;
                    ctl_roll.target_angle = ctl_roll.target_angle.max(ctl_roll.min_angle);
                }
            }
        }
        if keyboard.just_released(KeyCode::KeyD) {
            ctl_roll.target_angle = 0.0;
        }

        if keyboard.pressed(KeyCode::KeyA) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_roll.target_angle += ctl_roll.angle_rate;
                    ctl_roll.target_angle = ctl_roll.target_angle.min(ctl_roll.max_angle);
                }
            }
        }
        if keyboard.just_released(KeyCode::KeyA) {
            ctl_roll.target_angle = 0.0;
        }

        if keyboard.pressed(KeyCode::KeyQ) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_yaw.target_angle -= ctl_yaw.angle_rate;
                }
            }
        }

        if keyboard.pressed(KeyCode::KeyE) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_yaw.target_angle += ctl_yaw.angle_rate;
                }
            }
        }

        if keyboard.just_pressed(KeyCode::KeyR) {
            if *engine_state.get() == EngineState::Off {
                ctl_y.target_y = 0.0;
                ctl_pitch.target_angle = 0.0;
                ctl_roll.target_angle = 0.0;
                ctl_yaw.target_angle = 0.0;
            }
        }

        if keyboard.just_pressed(KeyCode::Escape) {
            // Exit the application
            std::process::exit(0);
        }
    }
}

pub fn engine_off(
    mut drone_query: Query<&mut ExternalForce, With<Drone>>,
    engine_state: Res<State<EngineState>>,
) {
    for mut force in drone_query.iter_mut() {
        if *engine_state.get() == EngineState::Off {
            force.force = Vec3::ZERO; // Stop the drone when engine is off
        }
    }
}

pub fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        DroneCamera,
        Camera3d::default(),
        Transform::from_xyz(0.0, FOLLOW_DIST, FOLLOW_DIST).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

pub fn update_camera_pos(
    drone_cam_params: Res<DroneCameraParams>,
    rapier_context: ReadRapierContext,
    drone_query: Query<&Transform, (With<Drone>, Without<DroneCamera>)>,
    mut cam_query: Query<&mut Transform, (With<DroneCamera>, Without<Drone>)>,
) {
    for mut cam_trans in cam_query.iter_mut() {
        for drone_trans in drone_query.iter() {
            let Ok(context) = rapier_context.single() else {
                continue;
            };

            let yaw = drone_cam_params.yaw;
            let pitch = drone_cam_params.pitch;
            let radius = drone_cam_params.radius;

            let x = radius * yaw.sin() * pitch.cos();
            let y = radius * pitch.sin();
            let z = radius * yaw.cos() * pitch.cos();

            let offset = Vec3::new(x, y, z);

            let drone_view_pos = drone_trans.translation + Vec3::Y;

            let ideal_camera_pos = drone_view_pos + offset;
            let dir = (ideal_camera_pos - drone_view_pos).normalize();
            let max_dist = offset.length();
            let mut final_dist = max_dist;

            if let Some((_entity, toi)) = context.cast_ray(
                drone_view_pos,
                dir,
                max_dist,
                true,
                QueryFilter::default().exclude_sensors(),
            ) {
                final_dist = toi - 0.1;
            }

            let actual_camera_pos = drone_view_pos + dir * final_dist;
            cam_trans.translation = actual_camera_pos;
            cam_trans.look_at(drone_view_pos, Vec3::Y);
        }
    }
}

pub fn control_camera_mouse(
    mouse_input: Res<ButtonInput<MouseButton>>,
    mut mouse_events: EventReader<MouseMotion>,
    mut cam_params: ResMut<DroneCameraParams>,
) {
    if mouse_input.pressed(MouseButton::Right) {
        for event in mouse_events.read() {
            cam_params.yaw -= event.delta.x * cam_params.sensitivity;
            cam_params.pitch -= event.delta.y * cam_params.sensitivity;

            // Limit pitch so it doesn’t flip
            cam_params.pitch = cam_params
                .pitch
                .clamp(-40_f32.to_radians(), 89_f32.to_radians());
        }
    }
}

pub fn spawn_ui(mut commands: Commands, asset_server: Res<AssetServer>) {
    let font = asset_server.load("./pixeloid_mono.ttf");

    commands
        .spawn((
            Node {
                width: Val::Percent(100.),
                height: Val::Percent(100.),
                flex_direction: FlexDirection::Column,
                align_items: AlignItems::FlexStart,
                justify_content: JustifyContent::FlexStart,
                ..Default::default()
            },
            BackgroundColor(Color::NONE),
        ))
        .with_children(|parent| {
            parent
                .spawn((
                    EngineUI,
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::srgba(0. / 255., 210. / 255., 0. / 255., 1.)),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        EngineText,
                        Text::new("Engine: On"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        OutputYText,
                        Text::new("Output Y: 0.00 m"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        TargetYText,
                        Text::new("Target Y: 0.00 m"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        OutputPitchText,
                        Text::new("Output Pitch: 0.00 deg"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        TargetPitchText,
                        Text::new("Target Pitch: 0.00 deg"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        OutputRollText,
                        Text::new("Output Roll: 0.00 deg"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        TargetRollText,
                        Text::new("Target Roll: 0.00 deg"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        OutputYawText,
                        Text::new("Output Yaw: 0.00 deg"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
                        width: Val::Px(380.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Start,
                        align_items: AlignItems::Center,
                        position_type: PositionType::Relative,
                        padding: UiRect {
                            left: Val::Px(8.),
                            right: Val::Px(8.),
                            top: Val::Px(8.),
                            bottom: Val::Px(8.),
                        },
                        border: UiRect {
                            left: Val::Px(2.),
                            right: Val::Px(2.),
                            top: Val::Px(2.),
                            bottom: Val::Px(2.),
                        },
                        ..Default::default()
                    },
                    BorderColor(Color::WHITE),
                    BackgroundColor(Color::BLACK),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        TargetYawText,
                        Text::new("Target Yaw: 0.00 deg"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 22.,
                            ..Default::default()
                        },
                    ));
                });
        });
}

pub fn update_engine_ui(
    engine_state: Res<State<EngineState>>,
    mut engine_ui_query: Query<&mut BackgroundColor, With<EngineUI>>,
    mut text_query: Query<&mut Text, With<EngineText>>,
) {
    for mut text in text_query.iter_mut() {
        if *engine_state.get() == EngineState::On {
            *text = "Engine: On".into();
            for mut ui in engine_ui_query.iter_mut() {
                *ui = BackgroundColor(Color::srgba(0. / 255., 210. / 255., 0. / 255., 1.));
            }
        } else {
            *text = "Engine: Off".into();
            for mut ui in engine_ui_query.iter_mut() {
                *ui = BackgroundColor(Color::srgba(210. / 255., 0. / 255., 0. / 255., 1.));
            }
        }
    }
}

pub fn update_output_y_text(
    drone_query: Query<&Transform, With<Drone>>,
    mut text_query: Query<&mut Text, With<OutputYText>>,
) {
    for tf in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Output Y: {:.2} m", tf.translation.y).into();
        }
    }
}

pub fn update_target_y_text(
    drone_query: Query<&HoverPid, With<Drone>>,
    mut text_query: Query<&mut Text, With<TargetYText>>,
) {
    for ctl_y in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Target Y: {:.2} m", ctl_y.target_y).into();
        }
    }
}

pub fn update_output_pitch_text(
    drone_query: Query<&Transform, With<Drone>>,
    mut text_query: Query<&mut Text, With<OutputPitchText>>,
) {
    for tf in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            let (_, x_rad, _) = tf.rotation.to_euler(EulerRot::YXZ);
            *text = format!("Output Pitch: {:.2} deg", x_rad.to_degrees()).into();
        }
    }
}

pub fn update_target_pitch_text(
    drone_query: Query<&PitchPid, With<Drone>>,
    mut text_query: Query<&mut Text, With<TargetPitchText>>,
) {
    for ctl_pitch in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!(
                "Target Pitch: {:.2} deg",
                ctl_pitch.target_angle.to_degrees()
            )
            .into();
        }
    }
}

pub fn update_output_roll_text(
    drone_query: Query<&Transform, With<Drone>>,
    mut text_query: Query<&mut Text, With<OutputRollText>>,
) {
    for tf in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            let (_, _, z_rad) = tf.rotation.to_euler(EulerRot::YXZ);
            *text = format!("Output Roll: {:.2} deg", z_rad.to_degrees()).into();
        }
    }
}

pub fn update_target_roll_text(
    drone_query: Query<&RollPid, With<Drone>>,
    mut text_query: Query<&mut Text, With<TargetRollText>>,
) {
    for roll in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Target Roll: {:.2} deg", roll.target_angle.to_degrees()).into();
        }
    }
}

pub fn update_output_yaw_text(
    drone_query: Query<&Transform, With<Drone>>,
    mut text_query: Query<&mut Text, With<OutputYawText>>,
) {
    for tf in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            let (y_rad, _, _) = tf.rotation.to_euler(EulerRot::YXZ);
            *text = format!("Output Yaw: {:.2} deg", y_rad.to_degrees()).into();
        }
    }
}

pub fn update_target_yaw_text(
    drone_query: Query<&YawPid, With<Drone>>,
    mut text_query: Query<&mut Text, With<TargetYawText>>,
) {
    for yaw in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Target Yaw: {:.2} deg", yaw.target_angle.to_degrees()).into();
        }
    }
}

pub fn spawn_light(mut commands: Commands) {
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 5000.0,
            ..Default::default()
        },
        Transform {
            rotation: Quat::from_rotation_x(-45.0_f32.to_radians())
                * Quat::from_rotation_y(30.0_f32.to_radians()),
            ..Default::default()
        },
    ));
}
