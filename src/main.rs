use bevy::{prelude::*, window::WindowMode};
use bevy_rapier3d::prelude::*;

const FOLLOW_DIST: f32 = 14.0;
const FOLLOW_HEIGHT: f32 = 6.0;
const LOOK_AHEAD: f32 = 8.0;
const SMOOTHNESS: f32 = 6.0;
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
    pub v_limit: f32,
    pub target_y: f32,
    pub min_y: f32,
    pub max_y: f32,
}

#[derive(Component)]
pub struct OutputYText;

#[derive(Component)]
pub struct TargetYText;

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
                update_engine_ui,
                update_output_y_text,
                update_target_y_text,
                update_camera_pos,
            ),
        )
        .add_systems(Update, hover.run_if(in_state(EngineState::On)))
        .run();
}

pub fn spawn_floor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(100., 0.1, 100.))),
            MeshMaterial3d(materials.add(Color::WHITE)),
            Transform::from_xyz(0.0, 0.0, 0.0),
        ))
        .insert(RigidBody::Fixed)
        .insert(Collider::cuboid(100. / 2., 0.1 / 2.0, 100. / 2.));
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
            LockedAxes::ROTATION_LOCKED,
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
            v_limit: 6.0,
            target_y: 0.0,
            min_y: 0.0,
            max_y: 120.0,
        });
}

pub fn hover(
    time: Res<Time>,
    mut drone_query: Query<
        (
            &Transform,
            &mut ExternalForce,
            &mut HoverPid,
            &ReadMassProperties,
        ),
        With<Drone>,
    >,
) {
    let dt = time.delta_secs();

    for (tf, mut force, mut ctl_y, m) in drone_query.iter_mut() {
        // y_0
        let y = tf.translation.y;

        // PID error
        let e = ctl_y.target_y - y;
        ctl_y.integral_e += e * dt;

        let norm_y = (y / ctl_y.max_y).clamp(0.0, 1.0);
        ctl_y.kp = ctl_y.min_kp + (ctl_y.max_kp - ctl_y.min_kp) * norm_y;

        let a_out =
            (ctl_y.kp * e) + (ctl_y.ki * ctl_y.integral_e) + (ctl_y.kd * (e - ctl_y.prev_e) / dt);

        // Update error
        ctl_y.prev_e = e;

        // Total desired vertical acceleration
        let force_y = m.mass * (a_out + GRAVITY);

        force.force = Vec3::new(0.0, force_y, 0.0);
    }
}

pub fn manual_control(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut drone_query: Query<&mut HoverPid, With<Drone>>,
    mut next_engine_state: ResMut<NextState<EngineState>>,
    engine_state: Res<State<EngineState>>,
    mut delay: ResMut<Delay>,
    time: Res<Time>,
) {
    for mut ctl_y in drone_query.iter_mut() {
        if keyboard.pressed(KeyCode::Space) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_y.target_y += 1.0;
                    ctl_y.target_y = ctl_y.target_y.min(ctl_y.max_y); // Prevent exceeding a maximum height
                }
            }
        }

        if keyboard.pressed(KeyCode::ControlLeft) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    ctl_y.target_y -= 1.0;
                    ctl_y.target_y = ctl_y.target_y.max(ctl_y.min_y); // Prevent going below ground level
                }
            }
        }

        if keyboard.just_pressed(KeyCode::KeyP) {
            if *engine_state.get() == EngineState::On {
                next_engine_state.set(EngineState::Off);

                ctl_y.target_y = 0.0;
            } else {
                next_engine_state.set(EngineState::On);

                ctl_y.target_y = ctl_y.min_y; // Default target height when engine is on
            }
        }

        if keyboard.just_pressed(KeyCode::Escape) {
            // Exit the application
            std::process::exit(0);
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
    time: Res<Time>,
    drone_query: Query<&Transform, (With<Drone>, Without<DroneCamera>)>,
    mut cam_query: Query<&mut Transform, (With<DroneCamera>, Without<Drone>)>,
) {
    for mut cam_trans in cam_query.iter_mut() {
        for drone_trans in drone_query.iter() {
            let fwd = drone_trans.forward();

            let desired = drone_trans.translation - fwd * FOLLOW_DIST + Vec3::Y * FOLLOW_HEIGHT;

            let alpha = 1.0 - (-SMOOTHNESS * time.delta_secs()).exp();
            cam_trans.translation = cam_trans.translation.lerp(desired, alpha);

            let target = drone_trans.translation + fwd * LOOK_AHEAD;
            cam_trans.look_at(target, Vec3::Y);
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
                        width: Val::Px(320.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Center,
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
                        width: Val::Px(320.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Center,
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
                        width: Val::Px(320.),
                        display: Display::Flex,
                        justify_content: JustifyContent::Center,
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
