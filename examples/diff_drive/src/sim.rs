//! Simulated differential-drive robot as a Copper bridge: the external
//! world side of the graph, like a motor driver plus odometry in hardware.

use crate::CuDiffDrive;
use crate::CuDiffDriveParams;
use crate::CuWheelSpeeds2;
use bincode::Decode;
use bincode::Encode;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::DecodeError;
use bincode::error::EncodeError;
use cu29::prelude::*;
use cu29::units::si::f64::Time;
use cu29::units::si::time::second;
use meander_core::CuPose2;
use meander_core::CuTwist2;
use serde::Deserialize;
use serde::Serialize;

tx_channels! {
    pub struct CuDiffDriveSimTxChannels: CuDiffDriveSimTxId {
        twist => CuTwist2,
    }
}

rx_channels! {
    pub struct CuDiffDriveSimRxChannels: CuDiffDriveSimRxId {
        pose => CuPose2,
    }
}

/// Configuration of [`CuDiffDriveSimBridge`], identical in Rust and RON.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuDiffDriveSimParams {
    /// Wheel geometry of the simulated base.
    pub drive: CuDiffDriveParams,
    /// Pose the robot starts from, in the map frame.
    pub start: CuPose2,
}

impl CuDiffDriveSimParams {
    /// Rejects invalid wheel geometry.
    pub fn validate(&self) -> CuResult<()> {
        self.drive.validate()
    }
}

/// A simulated differential-drive base behind a Copper bridge.
///
/// - Tx `twist`: a `CuTwist2` command advances the simulated pose by the
///   time elapsed since the previous command's `tov` (through wheel speeds,
///   so wheel geometry applies). The first command only sets the clock.
/// - Rx `pose`: the current `CuPose2` in the map frame, stamped with the
///   bridge's cycle time.
///
/// The pose and wheel speeds are bridge state: `freeze`/`thaw` round-trips
/// them, so replay resumes the robot where the keyframe left it.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct CuDiffDriveSimBridge {
    #[reflect(ignore)]
    model: CuDiffDrive,
    #[reflect(ignore)]
    last_tov: Option<CuTime>,
    #[reflect(ignore)]
    pose: CuPose2,
    #[reflect(ignore)]
    wheels: CuWheelSpeeds2,
}

impl CuDiffDriveSimBridge {
    /// Creates the bridge from validated params.
    pub fn from_params(params: CuDiffDriveSimParams) -> CuResult<Self> {
        params.validate()?;
        Ok(Self {
            model: CuDiffDrive::from_params(params.drive)?,
            last_tov: None,
            pose: params.start,
            wheels: CuWheelSpeeds2::default(),
        })
    }

    /// The simulated pose in the map frame.
    pub fn pose(&self) -> CuPose2 {
        self.pose
    }
}

impl Freezable for CuDiffDriveSimBridge {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.pose, encoder)?;
        Encode::encode(&self.wheels, encoder)?;
        Encode::encode(&self.last_tov.map(|t| t.as_nanos()), encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.pose = <CuPose2 as Decode<D::Context>>::decode(decoder)?;
        self.wheels = <CuWheelSpeeds2 as Decode<D::Context>>::decode(decoder)?;
        let nanos = <Option<u64> as Decode<D::Context>>::decode(decoder)?;
        self.last_tov = nanos.map(CuTime::from_nanos);
        Ok(())
    }
}

impl CuBridge for CuDiffDriveSimBridge {
    type Resources<'r> = ();
    type Tx = CuDiffDriveSimTxChannels;
    type Rx = CuDiffDriveSimRxChannels;

    fn new(
        cfg: Option<&ComponentConfig>,
        _tx: &[BridgeChannelConfig<CuDiffDriveSimTxId>],
        _rx: &[BridgeChannelConfig<CuDiffDriveSimRxId>],
        _res: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let params = cfg
            .map(ComponentConfig::deserialize_into::<CuDiffDriveSimParams>)
            .transpose()?
            .unwrap_or_default();
        Self::from_params(params)
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<CuDiffDriveSimTxId, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        match channel.id {
            CuDiffDriveSimTxId::Twist => {
                let msg = msg.downcast_ref::<CuTwist2>()?;
                if let Some(twist) = msg.payload() {
                    self.model.wheel_speeds(twist, &mut self.wheels);
                    if let Tov::Time(now) = msg.tov {
                        if let Some(last) = self.last_tov {
                            // Measured dt from the command's time of validity,
                            // clamped so a paused stream cannot teleport the robot.
                            let nanos = (now.as_nanos() as i64 - last.as_nanos() as i64)
                                .clamp(0, 100_000_000);
                            self.model.step(
                                &mut self.pose,
                                &self.wheels,
                                Time::new::<second>(nanos as f64 / 1e9),
                            );
                        }
                        self.last_tov = Some(now);
                    }
                }
                Ok(())
            }
        }
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<CuDiffDriveSimRxId, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        match channel.id {
            CuDiffDriveSimRxId::Pose => {
                let msg = msg.downcast_mut::<CuPose2>()?;
                msg.tov = Tov::Time(ctx.now());
                msg.set_payload(self.pose);
                Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::units::si::f64::Length;
    use cu29::units::si::f64::Velocity;
    use cu29::units::si::length::meter;
    use cu29::units::si::velocity::meter_per_second;

    fn bridge() -> CuDiffDriveSimBridge {
        CuDiffDriveSimBridge::from_params(CuDiffDriveSimParams::default()).unwrap()
    }

    fn cmd_at(nanos: u64) -> CuMsg<CuTwist2> {
        let mut cmd = CuMsg::new(Some(CuTwist2::new(
            Velocity::new::<meter_per_second>(0.5),
            Velocity::default(),
            Default::default(),
        )));
        cmd.tov = Tov::Time(CuTime::from_nanos(nanos));
        cmd
    }

    #[test]
    fn test_twist_advances_pose_and_pose_is_published() {
        let mut bridge = bridge();
        let (ctx, _clock) = CuContext::new_mock_clock();
        // First command only sets the clock; then 50 x 20 ms at 0.5 m/s.
        for i in 0..=50 {
            bridge
                .send(
                    &ctx,
                    &CuDiffDriveSimTxChannels::TWIST,
                    &cmd_at(i * 20_000_000),
                )
                .unwrap();
        }
        assert!((bridge.pose().x.get::<meter>() - 0.5).abs() < 1e-9);
        assert_eq!(bridge.pose().y, Length::default());

        let mut out = CuMsg::<CuPose2>::new(None);
        bridge
            .receive(&ctx, &CuDiffDriveSimRxChannels::POSE, &mut out)
            .unwrap();
        assert_eq!(out.payload(), Some(&bridge.pose()));
    }

    #[test]
    fn test_freeze_thaw_restores_pose() {
        let mut bridge = bridge();
        let (ctx, _clock) = CuContext::new_mock_clock();
        bridge
            .send(&ctx, &CuDiffDriveSimTxChannels::TWIST, &cmd_at(0))
            .unwrap();
        bridge
            .send(&ctx, &CuDiffDriveSimTxChannels::TWIST, &cmd_at(20_000_000))
            .unwrap();
        struct Frozen<'a, T: Freezable>(&'a T);
        impl<T: Freezable> Encode for Frozen<'_, T> {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                self.0.freeze(encoder)
            }
        }
        let bytes = bincode::encode_to_vec(Frozen(&bridge), bincode::config::standard()).unwrap();
        let mut replayed = self::bridge();
        let reader = bincode::de::read::SliceReader::new(&bytes);
        let mut decoder = bincode::de::DecoderImpl::new(reader, bincode::config::standard(), ());
        replayed.thaw(&mut decoder).unwrap();
        assert_eq!(replayed.pose(), bridge.pose());
    }

    #[test]
    fn test_bridge_reads_node_config() {
        let mut typo = ComponentConfig::new();
        typo.set("strat", 1.0f64);
        assert!(CuDiffDriveSimBridge::new(Some(&typo), &[], &[], ()).is_err());
        assert!(CuDiffDriveSimBridge::new(None, &[], &[], ()).is_ok());
    }

    #[test]
    fn test_long_pause_does_not_teleport() {
        let mut bridge = bridge();
        let (ctx, _clock) = CuContext::new_mock_clock();
        bridge
            .send(&ctx, &CuDiffDriveSimTxChannels::TWIST, &cmd_at(0))
            .unwrap();
        // 10 s gap: dt clamps to 100 ms, so at 0.5 m/s the robot moves 5 cm.
        bridge
            .send(
                &ctx,
                &CuDiffDriveSimTxChannels::TWIST,
                &cmd_at(10_000_000_000),
            )
            .unwrap();
        assert!((bridge.pose().x.get::<meter>() - 0.05).abs() < 1e-9);
    }
}
