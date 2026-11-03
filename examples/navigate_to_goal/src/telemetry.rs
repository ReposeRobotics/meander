//! App-local sink: streams the run to Studio over Server-Sent Events.
//!
//! Serves `GET /studio/describe` and `GET /events` on `<host>:<port>`.
//! The describe document is the app's self-description for the Studio
//! Copper viewer (semantic-kind streams, frames, Copper graph); it is
//! also sent as the first `describe` event on `/events`, followed by one
//! `map` event (costmap) and then `state` events (pose, twist, planned
//! path) whenever the robot moves. Studio renders all of it with generic
//! displays — no per-app Studio code.

use cu29::prelude::*;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;
use meander_core::CuPath2;
use meander_core::CuPose2;
use meander_core::CuTwist2;
use meander_costmap::CuCostmap2;
use meander_costmap::CuCostmapParams;
use serde::Deserialize;
use serde::Serialize;
use std::io::Read;
use std::io::Write;
use std::net::TcpListener;
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use crate::tasks::H;
use crate::tasks::MAX_PATH;
use crate::tasks::W;

/// Telemetry configuration, identical in Rust and RON.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuTelemetryParams {
    /// Nav2 map YAML whose inflated costmap is sent to viewers.
    pub map_path: String,
    /// Interface to bind. `127.0.0.1` is this machine only; `0.0.0.0`
    /// also serves other machines, e.g. over Tailscale.
    pub host: String,
    /// TCP port of the event stream.
    pub port: u16,
}

impl Default for CuTelemetryParams {
    fn default() -> Self {
        Self {
            map_path: "target/waypoint-mapping/map.yaml".into(),
            host: "127.0.0.1".into(),
            port: 18765,
        }
    }
}

#[derive(Default)]
struct Shared {
    seq: u64,
    pose: Option<(f64, f64, f64)>,
    twist: (f64, f64),
    planned: Vec<(f64, f64)>,
}

/// Streams pose, twist and the planned path to Studio; see the module docs.
///
/// - Input 0: `CuPose2` from the simulated base.
/// - Input 1: `CuPath2<MAX_PATH>` from the planner.
/// - Input 2: `CuTwist2` from the controller.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuTelemetrySink {
    #[reflect(ignore)]
    host: String,
    #[reflect(ignore)]
    port: u16,
    #[reflect(ignore)]
    map_event: String,
    #[reflect(ignore)]
    occupancy_event: String,
    #[reflect(ignore)]
    describe_json: String,
    #[reflect(ignore)]
    shared: Arc<Mutex<Shared>>,
    #[reflect(ignore)]
    running: Arc<AtomicBool>,
}

impl Freezable for CuTelemetrySink {}

impl CuTelemetrySink {
    fn state_event(shared: &Shared) -> String {
        let Some((x, y, theta)) = shared.pose else {
            return String::new();
        };
        let path: Vec<String> = shared
            .planned
            .iter()
            .map(|(px, py)| format!("[{px:.3},{py:.3}]"))
            .collect();
        format!(
            "event: state\ndata: {{\"seq\":{},\"pose\":[{x:.4},{y:.4},{theta:.4}],\"twist\":[{:.4},{:.4}],\"path\":[{}]}}\n\n",
            shared.seq,
            shared.twist.0,
            shared.twist.1,
            path.join(",")
        )
    }
}

/// Self-description for the Studio Copper viewer (`studio: 1`).
///
/// Stream ids match what Studio's SSE adapter publishes on its bus
/// (`costmap`, `pose`, `path`); the graph mirrors `copperconfig.ron`.
/// No controls are declared: the app accepts no uplink commands, and
/// Studio's control forms only render what is declared here.
fn describe_json() -> String {
    r##"{"studio":1,"app":{"id":"meander-nav","name":"Meander navigate-to-goal"},"frames":[{"id":"map"},{"id":"base_link","parent":"map"}],"assets":[],"graph":{"tasks":[{"id":"planner","type":"meander_planner::CuPlannerTask<200, 200, 2048>"},{"id":"controller","type":"meander_controller::CuPurePursuitTask<2048>"},{"id":"recorder","type":"tasks::CuNavigationRecorder"},{"id":"telemetry","type":"telemetry::CuTelemetrySink"}],"bridges":[{"id":"sim","type":"meander_example_diff_drive::CuDiffDriveSimBridge"},{"id":"telemetry-sse","type":"studio::SseBridge"}],"connections":[{"src":"sim/pose","dst":"planner","msg":"meander_core::CuPose2"},{"src":"sim/pose","dst":"controller","msg":"meander_core::CuPose2"},{"src":"planner","dst":"controller","msg":"meander_core::CuPath2<2048>"},{"src":"controller","dst":"sim/twist","msg":"meander_core::CuTwist2"},{"src":"sim/pose","dst":"recorder","msg":"meander_core::CuPose2"},{"src":"planner","dst":"recorder","msg":"meander_core::CuPath2<2048>"},{"src":"controller","dst":"recorder","msg":"meander_core::CuTwist2"},{"src":"sim/pose","dst":"telemetry","msg":"meander_core::CuPose2"},{"src":"planner","dst":"telemetry","msg":"meander_core::CuPath2<2048>"},{"src":"controller","dst":"telemetry","msg":"meander_core::CuTwist2"}]},"streams":[{"id":"occupancy","msg":"meander_grid::CuOccupancyGrid","kind":"occupancy_grid_3d","frame":"map","hints":{"in3d":true}},{"id":"costmap","msg":"meander_costmap::CuCostmap2","kind":"occupancy_grid","direction":"out","frame":"map","fields":[{"name":"width","type":"u32"},{"name":"height","type":"u32"},{"name":"resolution","type":"f64","unit":"m"},{"name":"origin","type":"vec2","unit":"m"},{"name":"costs","type":"u8[]"}],"hints":{"in3d":true}},{"id":"pose","msg":"meander_core::CuPose2","kind":"pose","direction":"out","frame":"map","fields":[{"name":"x","type":"f64","unit":"m"},{"name":"y","type":"f64","unit":"m"},{"name":"theta","type":"f64","unit":"rad"}],"hints":{"color":"#34d399","in3d":true}},{"id":"path","msg":"meander_core::CuPath2","kind":"path","direction":"out","frame":"map","fields":[{"name":"poses","type":"pose2[]","unit":"m"}],"hints":{"color":"#a78bfa","in3d":true}}],"controls":[]}"##
        .to_string()
}

impl CuSinkTask for CuTelemetrySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CuPose2, CuPath2<MAX_PATH>, CuTwist2);

    fn new(cfg: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuTelemetryParams>)
            .transpose()?
            .unwrap_or_default();
        if params.map_path.is_empty() || params.host.is_empty() || params.port == 0 {
            return Err(CuError::from(
                "telemetry: map_path and host must not be empty and port must be > 0",
            ));
        }
        let map = meander_grid::load_map::<W, H>(&params.map_path)?;
        let mut costmap =
            CuCostmap2::<W, H>::from_params(*map.geometry(), CuCostmapParams::default())?;
        costmap.update_from_occupancy(&map)?;
        let origin = map.geometry().origin;
        let mut costs = Vec::with_capacity(W * H);
        for row in costmap.costs() {
            costs.extend_from_slice(row);
        }
        let map_event = format!(
            "event: map\ndata: {{\"width\":{W},\"height\":{H},\"resolution\":{},\"origin\":[{},{}],\"costs\":\"{}\"}}\n\n",
            map.geometry().resolution.get::<meter>(),
            origin.x.get::<meter>(),
            origin.y.get::<meter>(),
            base64(&costs),
        );
        let occupancy: Vec<u8> = map
            .cells()
            .iter()
            .flatten()
            .map(|cell| match cell {
                meander_grid::CuOccupancy::Unknown => 127,
                meander_grid::CuOccupancy::Free => 0,
                meander_grid::CuOccupancy::Occupied => 255,
            })
            .collect();
        let occupancy_event = format!(
            "event: occupancy\ndata: {{\"width\":{W},\"height\":{H},\"resolution\":{},\"origin\":[{},{}],\"costs\":\"{}\"}}\n\n",
            map.geometry().resolution.get::<meter>(),
            origin.x.get::<meter>(),
            origin.y.get::<meter>(),
            base64(&occupancy),
        );
        Ok(Self {
            occupancy_event,
            host: params.host,
            port: params.port,
            map_event,
            describe_json: describe_json(),
            shared: Arc::new(Mutex::new(Shared::default())),
            running: Arc::new(AtomicBool::new(false)),
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        let listener = TcpListener::bind((self.host.as_str(), self.port))
            .map_err(|e| CuError::new_with_cause("telemetry: cannot bind event port", e))?;
        self.running.store(true, Ordering::Release);
        let (shared, running, map_event, occupancy_event, describe_json) = (
            Arc::clone(&self.shared),
            Arc::clone(&self.running),
            self.map_event.clone(),
            self.occupancy_event.clone(),
            self.describe_json.clone(),
        );
        std::thread::Builder::new()
            .name("navigate-telemetry".into())
            .spawn(move || {
                serve(
                    listener,
                    shared,
                    map_event,
                    occupancy_event,
                    describe_json,
                    running,
                )
            })
            .map_err(|e| CuError::new_with_cause("telemetry: cannot start server", e))?;
        println!(
            "telemetry on http://{}:{} (/studio/describe + /events for the Studio Copper viewer)",
            self.host, self.port
        );
        Ok(())
    }

    fn process<'i>(&mut self, _ctx: &CuContext, input: &Self::Input<'i>) -> CuResult<()> {
        let (pose_msg, path_msg, twist_msg) = *input;
        let mut shared = self
            .shared
            .lock()
            .map_err(|_| CuError::from("telemetry: state lock poisoned"))?;
        if let Some(path) = path_msg.payload()
            && !path.is_empty()
            && shared.planned.len() != path.len
        {
            shared.planned = path
                .poses()
                .iter()
                .map(|p| (p.x.get::<meter>(), p.y.get::<meter>()))
                .collect();
        }
        if let Some(twist) = twist_msg.payload() {
            shared.twist = (
                twist.linear_x.get::<meter_per_second>(),
                twist.angular.get::<radian_per_second>(),
            );
        }
        if let Some(pose) = pose_msg.payload() {
            shared.pose = Some((
                pose.x.get::<meter>(),
                pose.y.get::<meter>(),
                pose.theta.get::<cu29::units::si::angle::radian>(),
            ));
            shared.seq += 1;
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.running.store(false, Ordering::Release);
        Ok(())
    }
}

fn serve(
    listener: TcpListener,
    shared: Arc<Mutex<Shared>>,
    map_event: String,
    occupancy_event: String,
    describe_json: String,
    running: Arc<AtomicBool>,
) {
    for stream in listener.incoming() {
        if !running.load(Ordering::Acquire) {
            return;
        }
        let Ok(stream) = stream else { continue };
        let (shared, running, map_event, occupancy_event, describe_json) = (
            Arc::clone(&shared),
            Arc::clone(&running),
            map_event.clone(),
            occupancy_event.clone(),
            describe_json.clone(),
        );
        let _ = std::thread::spawn(move || {
            client(
                stream,
                shared,
                map_event,
                occupancy_event,
                describe_json,
                running,
            )
        });
    }
}

fn client(
    mut stream: TcpStream,
    shared: Arc<Mutex<Shared>>,
    map_event: String,
    occupancy_event: String,
    describe_json: String,
    running: Arc<AtomicBool>,
) {
    let _ = stream.set_nodelay(true);
    // Drain the request line/headers before replying.
    let mut request = [0u8; 1024];
    let _ = stream.read(&mut request);
    if request.starts_with(b"GET /studio/describe") {
        let _ = stream.write_all(
            format!(
                "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nAccess-Control-Allow-Origin: *\r\n\r\n{}",
                describe_json.len(),
                describe_json
            )
            .as_bytes(),
        );
        return;
    }
    if !request.starts_with(b"GET /events") {
        let _ = stream.write_all(
            b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nAccess-Control-Allow-Origin: *\r\n\r\n",
        );
        return;
    }
    // The description is the first event on the stream, then the map.
    let describe_event = format!("event: describe\ndata: {describe_json}\n\n");
    if stream
        .write_all(
            b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n\r\n",
        )
        .is_err()
        || stream.write_all(describe_event.as_bytes()).is_err()
        || stream.write_all(map_event.as_bytes()).is_err()
        || stream.write_all(occupancy_event.as_bytes()).is_err()
    {
        return;
    }
    let mut sent = 0u64;
    while running.load(Ordering::Acquire) {
        let event = {
            let Ok(shared) = shared.lock() else { return };
            (shared.seq != sent).then(|| CuTelemetrySink::state_event(&shared))
        };
        if let Some(event) = event
            && !event.is_empty()
        {
            if stream.write_all(event.as_bytes()).is_err() {
                return;
            }
            sent += 1;
            // `sent` counts events, `seq` counts poses; resync cheaply:
            if let Ok(shared) = shared.lock() {
                sent = shared.seq;
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
}

/// Standard base64, no line breaks.
fn base64(data: &[u8]) -> String {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let b = [
            chunk[0] as u32,
            *chunk.get(1).unwrap_or(&0) as u32,
            *chunk.get(2).unwrap_or(&0) as u32,
        ];
        let n = (b[0] << 16) | (b[1] << 8) | b[2];
        out.push(ALPHABET[(n >> 18) as usize & 63] as char);
        out.push(ALPHABET[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 {
            ALPHABET[(n >> 6) as usize & 63] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            ALPHABET[n as usize & 63] as char
        } else {
            '='
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_base64_matches_rfc4648() {
        assert_eq!(base64(b""), "");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
        assert_eq!(base64(&[254, 255]), "/v8=");
    }

    #[test]
    fn test_describe_matches_studio_contract() {
        let json = describe_json();
        // Studio's validateDescription: studio==1, app.id, stream id+kind.
        assert!(json.starts_with("{\"studio\":1,"));
        assert!(json.contains("\"app\":{\"id\":\"meander-nav\""));
        // Stream ids are the ones Studio's SSE adapter publishes on its bus.
        for stream in ["\"id\":\"costmap\"", "\"id\":\"pose\"", "\"id\":\"path\""] {
            assert!(json.contains(stream), "missing stream {stream}");
        }
        for kind in [
            "\"kind\":\"occupancy_grid\"",
            "\"kind\":\"pose\"",
            "\"kind\":\"path\"",
        ] {
            assert!(json.contains(kind), "missing kind {kind}");
        }
        // No uplink: nothing declared, so Studio renders no control forms.
        assert!(json.contains("\"controls\":[]"));
        // The graph mirrors every cnx in copperconfig.ron, recorder included.
        for edge in [
            "\"src\":\"sim/pose\",\"dst\":\"recorder\"",
            "\"src\":\"planner\",\"dst\":\"recorder\"",
            "\"src\":\"controller\",\"dst\":\"recorder\"",
        ] {
            assert!(json.contains(edge), "missing graph edge {edge}");
        }
    }

    #[test]
    fn test_state_event_shape() {
        let shared = Shared {
            seq: 3,
            pose: Some((1.0, -2.0, 0.5)),
            twist: (0.25, -0.5),
            planned: vec![(0.0, 0.0), (1.0, 1.0)],
        };
        let event = CuTelemetrySink::state_event(&shared);
        assert!(event.starts_with("event: state\ndata: {"));
        assert!(event.contains("\"pose\":[1.0000,-2.0000,0.5000]"));
        assert!(event.contains("\"path\":[[0.000,0.000],[1.000,1.000]]"));
        assert_eq!(CuTelemetrySink::state_event(&Shared::default()), "");
    }
}
