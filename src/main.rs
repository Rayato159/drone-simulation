use bevy::{prelude::*, window::WindowMode};
use bevy_rapier3d::prelude::*;

const FOLLOW_DIST: f32 = 14.0;
const FOLLOW_HEIGHT: f32 = 6.0;
const LOOK_AHEAD: f32 = 8.0;
const SMOOTHNESS: f32 = 6.0;
const SAFETY_FACTOR: f32 = 0.8;

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
pub struct CruiseZPid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub prev_e: f32,
    pub integral_e: f32,
    pub a_limit: f32,
    pub target_v: f32,
    pub max_v: f32,
    pub min_v: f32,
}

#[derive(Component)]
pub struct CruiseXPid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub prev_e: f32,
    pub integral_e: f32,
    pub a_limit: f32,
    pub target_v: f32,
    pub max_v: f32,
    pub min_v: f32,
}

#[derive(Component)]
pub struct OutputYText;

#[derive(Component)]
pub struct TargetYText;

#[derive(Component)]
pub struct OutputZText;

#[derive(Component)]
pub struct TargetZText;

#[derive(Component)]
pub struct OutputXText;

#[derive(Component)]
pub struct TargetXText;

#[derive(Component)]
pub struct HoverModeText;

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

#[derive(Resource)]
pub struct Dir(pub Vec3);

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
        .insert_resource(Dir(Vec3::ZERO))
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
                update_movement,
                update_engine_ui,
                update_output_y_text,
                update_target_y_text,
                update_output_z_text,
                update_target_z_text,
                update_output_x_text,
                update_target_x_text,
                update_camera_pos,
            ),
        )
        .add_systems(
            Update,
            (hover, cruise_z, cruise_x).run_if(in_state(EngineState::On)),
        )
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
            RigidBody::Dynamic,
            Transform::from_xyz(0.0, 3.0, 0.0),
            Collider::cuboid(1.0 / 2.0, 0.5 / 2.0, 1.0 / 2.0),
            GravityScale(1.0),
            LockedAxes::ROTATION_LOCKED,
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
        })
        .insert(CruiseZPid {
            kp: 3.0,
            ki: 0.2,
            kd: 1.0,
            prev_e: 0.0,
            integral_e: 0.0,
            a_limit: 9.0,
            target_v: 0.0,
            max_v: 18.0,
            min_v: -18.0,
        })
        .insert(CruiseXPid {
            kp: 3.0,
            ki: 0.2,
            kd: 1.0,
            prev_e: 0.0,
            integral_e: 0.0,
            a_limit: 9.0,
            target_v: 0.0,
            max_v: 18.0,
            min_v: -18.0,
        });
}

pub fn hover(
    time: Res<Time>,
    mut drone_query: Query<(&Transform, &mut Velocity, &mut HoverPid), With<Drone>>,
) {
    let dt = time.delta_secs();

    for (tf, mut vel, mut ctl_y) in drone_query.iter_mut() {
        // y_0
        let y = tf.translation.y;

        // Error
        let e = ctl_y.target_y - y;

        // Integral error
        ctl_y.integral_e += e * dt;

        let norm_y = (y / ctl_y.max_y).clamp(0.0, 1.0);
        ctl_y.kp = ctl_y.min_kp + (ctl_y.max_kp - ctl_y.min_kp) * norm_y;

        // PID fully computed
        let mut v_out =
            (ctl_y.kp * e) + (ctl_y.ki * ctl_y.integral_e) + (ctl_y.kd * (e - ctl_y.prev_e) / dt);
        v_out = v_out.clamp(-ctl_y.v_limit, ctl_y.v_limit);

        // Update variables
        vel.linvel.y = vel.linvel.y + v_out * SAFETY_FACTOR;
        ctl_y.prev_e = e;
    }
}

pub fn cruise_z(
    time: Res<Time>,
    mut drone_query: Query<(&mut Velocity, &mut CruiseZPid), With<Drone>>,
) {
    let dt = time.delta_secs();

    for (mut vel, mut ctl_z) in drone_query.iter_mut() {
        // z_0
        let v = vel.linvel.z;

        // Error
        let e = ctl_z.target_v - v;

        // Integral error
        ctl_z.integral_e += e * dt;

        // PID fully computed
        let mut a_out =
            (ctl_z.kp * e) + (ctl_z.ki * ctl_z.integral_e) + (ctl_z.kd * (e - ctl_z.prev_e) / dt);
        a_out = a_out.clamp(-ctl_z.a_limit, ctl_z.a_limit);

        // Update variables
        vel.linvel.z = vel.linvel.z + a_out * dt;
        ctl_z.prev_e = e;
    }
}

pub fn cruise_x(
    time: Res<Time>,
    mut drone_query: Query<(&mut Velocity, &mut CruiseXPid), With<Drone>>,
) {
    let dt = time.delta_secs();

    for (mut vel, mut ctl_x) in drone_query.iter_mut() {
        // z_0
        let v = vel.linvel.x;

        // Error
        let e = ctl_x.target_v - v;

        // Integral error
        ctl_x.integral_e += e * dt;

        // PID fully computed
        let mut a_out =
            (ctl_x.kp * e) + (ctl_x.ki * ctl_x.integral_e) + (ctl_x.kd * (e - ctl_x.prev_e) / dt);
        a_out = a_out.clamp(-ctl_x.a_limit, ctl_x.a_limit);

        // Update variables
        vel.linvel.x = vel.linvel.x + a_out * dt;
        ctl_x.prev_e = e;
    }
}

pub fn manual_control(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut drone_query: Query<(&mut HoverPid, &mut CruiseZPid, &mut CruiseXPid), With<Drone>>,
    mut next_engine_state: ResMut<NextState<EngineState>>,
    engine_state: Res<State<EngineState>>,
    mut delay: ResMut<Delay>,
    time: Res<Time>,
    mut dir: ResMut<Dir>,
) {
    for (mut ctl_y, mut ctl_z, mut ctl_x) in drone_query.iter_mut() {
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

        if keyboard.pressed(KeyCode::KeyW) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    dir.0 = Vec3::new(0.0, 0.0, -1.0);
                }
            }
        } else if keyboard.just_released(KeyCode::KeyW) {
            dir.0 = Vec3::ZERO;
        }

        if keyboard.pressed(KeyCode::KeyS) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    dir.0 = Vec3::new(0.0, 0.0, 1.0);
                }
            }
        } else if keyboard.just_released(KeyCode::KeyS) {
            dir.0 = Vec3::ZERO;
        }

        if keyboard.pressed(KeyCode::KeyA) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    dir.0 = Vec3::new(-1.0, 0.0, 0.0);
                }
            }
        } else if keyboard.just_released(KeyCode::KeyA) {
            dir.0 = Vec3::ZERO;
        }

        if keyboard.pressed(KeyCode::KeyD) {
            delay.timer.tick(time.delta());

            if delay.timer.just_finished() {
                if *engine_state.get() == EngineState::On {
                    dir.0 = Vec3::new(1.0, 0.0, 0.0);
                }
            }
        } else if keyboard.just_released(KeyCode::KeyD) {
            dir.0 = Vec3::ZERO;
        }

        if keyboard.just_pressed(KeyCode::KeyP) {
            if *engine_state.get() == EngineState::On {
                next_engine_state.set(EngineState::Off);

                ctl_y.target_y = 0.0;
                ctl_z.target_v = 0.0;
                ctl_x.target_v = 0.0;
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

pub fn update_movement(
    mut drone_query: Query<(&mut CruiseZPid, &mut CruiseXPid), With<Drone>>,
    dir: Res<Dir>,
) {
    if dir.0.length_squared() > 0.0 {
        for (mut ctl_z, mut ctl_x) in drone_query.iter_mut() {
            let norm_dir = dir.0.normalize();

            ctl_z.target_v += norm_dir.z;
            ctl_z.target_v = ctl_z.target_v.clamp(ctl_z.min_v, ctl_z.max_v);

            ctl_x.target_v += norm_dir.x;
            ctl_x.target_v = ctl_x.target_v.clamp(ctl_x.min_v, ctl_x.max_v);
        }
    } else {
        for (mut ctl_z, mut ctl_x) in drone_query.iter_mut() {
            ctl_z.target_v = 0.0;
            ctl_x.target_v = 0.0;
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
                        HoverModeText,
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
                        OutputZText,
                        Text::new("Output Z: 0.00 m/s"),
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
                        TargetZText,
                        Text::new("Target Z: 0.00 m/s"),
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
                        OutputXText,
                        Text::new("Output X: 0.00 m/s"),
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
                        TargetXText,
                        Text::new("Target X: 0.00 m/s"),
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
    mut text_query: Query<&mut Text, With<HoverModeText>>,
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

pub fn update_output_z_text(
    drone_query: Query<&Velocity, With<Drone>>,
    mut text_query: Query<&mut Text, With<OutputZText>>,
) {
    for vel in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Output Z: {:.2} m/s", vel.linvel.z).into();
        }
    }
}

pub fn update_target_z_text(
    drone_query: Query<&CruiseZPid, With<Drone>>,
    mut text_query: Query<&mut Text, With<TargetZText>>,
) {
    for ctl_z in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Target Z: {:.2} m/s", ctl_z.target_v).into();
        }
    }
}

pub fn update_output_x_text(
    drone_query: Query<&Velocity, With<Drone>>,
    mut text_query: Query<&mut Text, With<OutputXText>>,
) {
    for vel in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Output X: {:.2} m/s", vel.linvel.x).into();
        }
    }
}

pub fn update_target_x_text(
    drone_query: Query<&CruiseXPid, With<Drone>>,
    mut text_query: Query<&mut Text, With<TargetXText>>,
) {
    for ctl_x in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Target X: {:.2} m/s", ctl_x.target_v).into();
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
