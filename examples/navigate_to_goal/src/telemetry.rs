//! App-local sink: streams the run to Studio over Server-Sent Events.
//!
//! Serves `GET /events` on `127.0.0.1:<port>`: one `map` event (costmap,
//! start, goal) when a client connects, then `state` events (pose, twist,
//! planned path) whenever the robot moves. Studio's Drive tab connects
//! here in Live mode and drives its 3D robot from these events.

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
        Ok(Self {
            host: params.host,
            port: params.port,
            map_event,
            shared: Arc::new(Mutex::new(Shared::default())),
            running: Arc::new(AtomicBool::new(false)),
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        let listener = TcpListener::bind((self.host.as_str(), self.port))
            .map_err(|e| CuError::new_with_cause("telemetry: cannot bind event port", e))?;
        self.running.store(true, Ordering::Release);
        let (shared, running, map_event) = (
            Arc::clone(&self.shared),
            Arc::clone(&self.running),
            self.map_event.clone(),
        );
        std::thread::Builder::new()
            .name("navigate-telemetry".into())
            .spawn(move || serve(listener, shared, map_event, running))
            .map_err(|e| CuError::new_with_cause("telemetry: cannot start server", e))?;
        println!(
            "telemetry on http://{}:{}/events (Studio Drive tab, Live mode)",
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
    running: Arc<AtomicBool>,
) {
    for stream in listener.incoming() {
        if !running.load(Ordering::Acquire) {
            return;
        }
        let Ok(stream) = stream else { continue };
        let (shared, running, map_event) =
            (Arc::clone(&shared), Arc::clone(&running), map_event.clone());
        let _ = std::thread::spawn(move || client(stream, shared, map_event, running));
    }
}

fn client(
    mut stream: TcpStream,
    shared: Arc<Mutex<Shared>>,
    map_event: String,
    running: Arc<AtomicBool>,
) {
    let _ = stream.set_nodelay(true);
    // Drain the request line/headers before replying.
    let mut request = [0u8; 1024];
    let _ = stream.read(&mut request);
    if !request.starts_with(b"GET /events") {
        let _ = stream.write_all(
            b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nAccess-Control-Allow-Origin: *\r\n\r\n",
        );
        return;
    }
    if stream
        .write_all(
            b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n\r\n",
        )
        .is_err()
        || stream.write_all(map_event.as_bytes()).is_err()
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
