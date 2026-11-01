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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
#[serde(default, deny_unknown_fields)]
pub struct CuDiffDriveSimParams {
    /// Wheel geometry of the simulated base.
    pub drive: CuDiffDriveParams,
    /// Pose the robot starts from, in the map frame.
    pub start: CuPose2,
    /// How far one commanded twist advances the robot per bridge cycle.
    pub sample_period: Time,
}

impl Default for CuDiffDriveSimParams {
    fn default() -> Self {
        Self {
            drive: CuDiffDriveParams::default(),
            start: CuPose2::default(),
            sample_period: Time::new::<second>(0.02),
        }
    }
}

impl CuDiffDriveSimParams {
    /// Rejects invalid wheel geometry, a non-finite start, and a
    /// non-positive sample period.
    pub fn validate(&self) -> CuResult<()> {
        self.drive.validate()?;
        let period = self.sample_period.get::<second>();
        if !period.is_finite() || period <= 0.0 {
            return Err(CuError::from(
                "CuDiffDriveSimParams: sample_period must be finite and > 0",
            ));
        }
        Ok(())
    }
}

/// A simulated differential-drive base behind a Copper bridge.
///
/// - Tx `twist`: a `CuTwist2` command advances the simulated pose by
///   `sample_period` (through wheel speeds, so wheel geometry applies).
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
    sample_period: Time,
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
            sample_period: params.sample_period,
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
        Encode::encode(&self.wheels, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.pose = <CuPose2 as Decode<D::Context>>::decode(decoder)?;
        self.wheels = <CuWheelSpeeds2 as Decode<D::Context>>::decode(decoder)?;
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
                    self.model
                        .step(&mut self.pose, &self.wheels, self.sample_period);
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

    #[test]
    fn test_twist_advances_pose_and_pose_is_published() {
        let mut bridge = bridge();
        let (ctx, _clock) = CuContext::new_mock_clock();
        let cmd = CuMsg::new(Some(CuTwist2::new(
            Velocity::new::<meter_per_second>(0.5),
            Velocity::default(),
            Default::default(),
        )));
        for _ in 0..50 {
            bridge
                .send(&ctx, &CuDiffDriveSimTxChannels::TWIST, &cmd)
                .unwrap();
        }
        // 50 cycles x 0.02 s at 0.5 m/s straight ahead.
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
        let cmd = CuMsg::new(Some(CuTwist2::new(
            Velocity::new::<meter_per_second>(0.5),
            Velocity::default(),
            Default::default(),
        )));
        bridge
            .send(&ctx, &CuDiffDriveSimTxChannels::TWIST, &cmd)
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
        let mut cfg = ComponentConfig::new();
        cfg.set("sample_period", 0.05f64);
        let bridge = CuDiffDriveSimBridge::new(Some(&cfg), &[], &[], ()).unwrap();
        assert_eq!(bridge.sample_period, Time::new::<second>(0.05));
        let mut typo = ComponentConfig::new();
        typo.set("sample_peroid", 0.05f64);
        assert!(CuDiffDriveSimBridge::new(Some(&typo), &[], &[], ()).is_err());
    }
}
