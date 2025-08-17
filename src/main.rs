use bevy::{prelude::*, window::WindowMode};
use bevy_rapier3d::prelude::*;

const FOLLOW_DIST: f32 = 14.0;
const FOLLOW_HEIGHT: f32 = 6.0;
const LOOK_AHEAD: f32 = 8.0;
const SMOOTHNESS: f32 = 6.0;

#[derive(Component)]
pub struct Drone;

#[derive(Component)]
pub struct DroneCamera;

#[derive(Component, Default)]
pub struct AltitudeState {
    pub vy: f32,
}

#[derive(Component)]
pub struct AltVelPid {
    pub kp: f32,
    pub ki: f32,
    pub kv: f32,
    pub kd: f32,
    pub prev_e: f32,
    pub integral_e: f32,
    pub v_limit: f32,
    pub target_y: f32,
}

#[derive(Component)]
pub struct OutputYText;

#[derive(Component)]
pub struct LimitYText;

#[derive(Component)]
pub struct PIDText;

#[derive(Component)]
pub struct PidUI;

#[derive(States, Default, Debug, Clone, PartialEq, Eq, Hash)]
pub enum PIDState {
    On,
    #[default]
    Off,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "PID Sim".into(),
                resizable: true,
                mode: WindowMode::BorderlessFullscreen(MonitorSelection::Primary),
                ..Default::default()
            }),
            ..Default::default()
        }))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .init_state::<PIDState>()
        .add_systems(Startup, spawn_floor)
        .add_systems(Startup, spawn_drone)
        .add_systems(Startup, spawn_light)
        .add_systems(Startup, spawn_camera)
        .add_systems(Startup, spawn_ui)
        .add_systems(
            Update,
            (
                manual_thrust_input,
                update_pid_ui,
                update_output_y_text,
                update_limit_y_text,
                update_camera_pos,
            ),
        )
        .add_systems(Update, pid_altitude_system.run_if(in_state(PIDState::On)))
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
    commands.spawn((
        Drone,
        Mesh3d(meshes.add(Cuboid::new(1.0, 0.5, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        RigidBody::Dynamic,
        Transform::from_xyz(0.0, 3.0, 0.0),
        Collider::cuboid(1.0 / 2.0, 0.5 / 2.0, 1.0 / 2.0),
        GravityScale(1.0),
        LockedAxes::ROTATION_LOCKED,
        AltitudeState::default(),
        Velocity::zero(),
        AltVelPid {
            kp: 2.0,
            ki: 0.1,
            kd: 0.5,
            kv: 8.0,
            prev_e: 0.0,
            integral_e: 0.0,
            v_limit: 4.0,
            target_y: 10.0,
        },
    ));
}

pub fn pid_altitude_system(
    time: Res<Time>,
    mut drone_query: Query<
        (
            &Transform,
            &mut Velocity,
            &mut AltVelPid,
            &mut AltitudeState,
        ),
        With<Drone>,
    >,
) {
    let dt = time.delta_secs().max(1e-4);

    for (tf, mut vel, mut ctl, mut st) in drone_query.iter_mut() {
        // y_0
        let y = tf.translation.y;

        // Error
        let e = ctl.target_y - y;

        // Integral error
        ctl.integral_e += e * dt;

        // PID fully computed
        let mut v_out = (ctl.kp * e) + (ctl.ki * ctl.integral_e) + (ctl.kd * (e - ctl.prev_e) / dt);
        v_out = v_out.clamp(-ctl.v_limit, ctl.v_limit);

        // Prevent overshoot
        let alpha = 1.0 - (-ctl.kv * dt).exp();

        // Update variables
        st.vy = st.vy * (1.0 - alpha) + v_out * alpha;

        vel.linvel.y = st.vy;
        ctl.prev_e = e;
    }
}

pub fn manual_thrust_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut drone_query: Query<(&mut Velocity, &mut AltVelPid), With<Drone>>,
    mut next_auto_pilot_state: ResMut<NextState<PIDState>>,
    auto_pilot_state: Res<State<PIDState>>,
) {
    for (mut vel, mut ctl) in drone_query.iter_mut() {
        if keyboard.pressed(KeyCode::Space) {
            vel.linvel.y = ctl.v_limit;
        }

        if keyboard.pressed(KeyCode::ControlLeft) {
            vel.linvel.y = -ctl.v_limit;
        }

        if keyboard.just_pressed(KeyCode::ArrowUp) {
            ctl.target_y += 1.0;
            ctl.target_y = ctl.target_y.min(120.0); // Prevent exceeding a maximum height
        }

        if keyboard.just_pressed(KeyCode::ArrowDown) {
            ctl.target_y -= 1.0;
            ctl.target_y = ctl.target_y.max(0.0); // Prevent negative target height
        }

        if keyboard.just_pressed(KeyCode::KeyP) {
            if *auto_pilot_state.get() == PIDState::On {
                next_auto_pilot_state.set(PIDState::Off);
            } else {
                next_auto_pilot_state.set(PIDState::On);
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
                flex_direction: FlexDirection::Row,
                align_items: AlignItems::FlexEnd,
                justify_content: JustifyContent::FlexStart,
                row_gap: Val::Px(24.),
                ..Default::default()
            },
            BackgroundColor(Color::NONE),
        ))
        .with_children(|parent| {
            parent
                .spawn((
                    PidUI,
                    Node {
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
                        PIDText,
                        Text::new("PID: On"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 28.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
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
                            font_size: 28.,
                            ..Default::default()
                        },
                    ));
                });
        })
        .with_children(|parent| {
            parent
                .spawn((
                    Node {
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
                        LimitYText,
                        Text::new("Target Y: 30.00 m"),
                        TextColor(Color::WHITE),
                        TextLayout::new_with_justify(JustifyText::Left),
                        TextFont {
                            font: font.clone(),
                            font_size: 28.,
                            ..Default::default()
                        },
                    ));
                });
        });
}

pub fn update_pid_ui(
    auto_pilot_state: Res<State<PIDState>>,
    mut pid_ui_query: Query<&mut BackgroundColor, With<PidUI>>,
    mut text_query: Query<&mut Text, With<PIDText>>,
) {
    for mut text in text_query.iter_mut() {
        if *auto_pilot_state.get() == PIDState::On {
            *text = "PID: On".into();
            for mut ui in pid_ui_query.iter_mut() {
                *ui = BackgroundColor(Color::srgba(0. / 255., 210. / 255., 0. / 255., 1.));
            }
        } else {
            *text = "PID: Off".into();
            for mut ui in pid_ui_query.iter_mut() {
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

pub fn update_limit_y_text(
    drone_query: Query<&AltVelPid, With<Drone>>,
    mut text_query: Query<&mut Text, With<LimitYText>>,
) {
    for ctl in drone_query.iter() {
        for mut text in text_query.iter_mut() {
            *text = format!("Target Y: {:.2} m", ctl.target_y).into();
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
