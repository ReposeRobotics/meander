//! ROS map_server trinary P5 images and their YAML sidecars.

use crate::CuCell2;
use crate::CuGridGeometry2;
use crate::CuOccupancy;
use crate::CuOccupancyGrid;
use alloc::format;
use alloc::string::String;
use alloc::string::ToString;
use core::fmt::Write;
use cu29::prelude::CuError;
use cu29::prelude::CuResult;
use cu29::units::si::angle::radian;
use cu29::units::si::f64::Angle;
use cu29::units::si::f64::Length;
use cu29::units::si::length::meter;
use meander_core::CuPose2;
use serde::Deserialize;
use serde::Serialize;

/// Supported map_server interpretation of image pixels.
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CuMapMode {
    /// Threshold pixels into occupied, free and unknown cells.
    #[default]
    Trinary,
}

/// ROS map_server YAML metadata, with SI units in Rust and scalar SI values on disk.
///
/// Text conversion accepts the flat mapping emitted by map_saver: scalar fields,
/// an inline three-element origin, comments, and plain or quoted image paths.
/// It rejects duplicate/unknown keys and unsupported YAML constructs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "MetadataWire", into = "MetadataWire")]
pub struct CuMapMetadata {
    /// Image path, resolved relative to the YAML file by `load_map` (with `std`).
    pub image: String,
    /// Side length of one cell.
    pub resolution: Length,
    /// World pose of the outer corner of cell (0, 0).
    pub origin: CuPose2,
    /// Interpret bright pixels as occupied when true.
    pub negate: bool,
    /// Dimensionless occupancy probability at or above which a cell is occupied.
    pub occupied_thresh: f64,
    /// Dimensionless occupancy probability at or below which a cell is free.
    pub free_thresh: f64,
    /// Image interpretation.
    pub mode: CuMapMode,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct MetadataWire {
    image: String,
    resolution: f64,
    origin: [f64; 3],
    negate: u8,
    occupied_thresh: f64,
    free_thresh: f64,
    #[serde(default)]
    mode: CuMapMode,
}

impl TryFrom<MetadataWire> for CuMapMetadata {
    type Error = CuError;
    fn try_from(w: MetadataWire) -> CuResult<Self> {
        if w.negate > 1 {
            return Err(error("negate must be 0 or 1"));
        }
        let metadata = Self {
            image: w.image,
            resolution: Length::new::<meter>(w.resolution),
            // Preserve yaw, including nonfinite values, for validation.
            origin: CuPose2 {
                x: Length::new::<meter>(w.origin[0]),
                y: Length::new::<meter>(w.origin[1]),
                theta: Angle::new::<radian>(w.origin[2]),
            },
            negate: w.negate == 1,
            occupied_thresh: w.occupied_thresh,
            free_thresh: w.free_thresh,
            mode: w.mode,
        };
        metadata.validate()?;
        Ok(metadata)
    }
}

impl From<CuMapMetadata> for MetadataWire {
    fn from(m: CuMapMetadata) -> Self {
        Self {
            image: m.image,
            resolution: m.resolution.get::<meter>(),
            origin: [
                m.origin.x.get::<meter>(),
                m.origin.y.get::<meter>(),
                m.origin.theta.get::<radian>(),
            ],
            negate: u8::from(m.negate),
            occupied_thresh: m.occupied_thresh,
            free_thresh: m.free_thresh,
            mode: m.mode,
        }
    }
}

impl CuMapMetadata {
    /// Creates metadata with ROS map defaults (free 0.196, occupied 0.65).
    pub fn new(image: String, geometry: &CuGridGeometry2) -> CuResult<Self> {
        let result = Self {
            image,
            resolution: geometry.resolution,
            origin: CuPose2::new(geometry.origin.x, geometry.origin.y, Angle::default()),
            negate: false,
            occupied_thresh: 0.65,
            free_thresh: 0.196,
            mode: CuMapMode::Trinary,
        };
        result.validate()?;
        Ok(result)
    }

    /// Checks finite geometry and `0 <= free_thresh < occupied_thresh <= 1`.
    /// Nonzero yaw is valid metadata but cannot be decoded into an axis-aligned grid.
    pub fn validate(&self) -> CuResult<()> {
        CuGridGeometry2::new(self.resolution, self.origin.position())?;
        if self.image.is_empty()
            || self.image.chars().any(char::is_control)
            || !self.origin.theta.get::<radian>().is_finite()
        {
            return Err(error(
                "image must be nonempty without control characters; yaw must be finite",
            ));
        }
        if !self.free_thresh.is_finite()
            || !self.occupied_thresh.is_finite()
            || self.free_thresh < 0.0
            || self.free_thresh >= self.occupied_thresh
            || self.occupied_thresh > 1.0
        {
            return Err(error("require 0 <= free_thresh < occupied_thresh <= 1"));
        }
        Ok(())
    }

    /// Emits a YAML sidecar. Allocates; use outside the real-time path.
    pub fn to_yaml(&self) -> CuResult<String> {
        self.validate()?;
        Ok(format!(
            "image: '{}'\nresolution: {}\norigin: [{}, {}, {}]\nnegate: {}\noccupied_thresh: {}\nfree_thresh: {}\nmode: trinary\n",
            self.image.replace('\'', "''"),
            self.resolution.get::<meter>(),
            self.origin.x.get::<meter>(),
            self.origin.y.get::<meter>(),
            self.origin.theta.get::<radian>(),
            u8::from(self.negate),
            self.occupied_thresh,
            self.free_thresh
        ))
    }

    /// Parses the map_saver YAML subset described on [`CuMapMetadata`]. Allocates.
    pub fn from_yaml(text: &str) -> CuResult<Self> {
        let mut fields: [Option<&str>; 7] = [None; 7];
        let mut started = false;
        let mut ended = false;
        for line in text.lines() {
            let line = uncomment(line)?.trim();
            if line.is_empty() {
                continue;
            }
            if ended || (line == "---" && started) {
                return Err(error("expected a single YAML document"));
            }
            if line == "..." {
                ended = true;
                continue;
            }
            started = true;
            if line == "---" {
                continue;
            }
            let (key, value) = line
                .split_once(':')
                .ok_or_else(|| error("expected YAML key: value"))?;
            let index = match key.trim() {
                "image" => 0,
                "resolution" => 1,
                "origin" => 2,
                "negate" => 3,
                "occupied_thresh" => 4,
                "free_thresh" => 5,
                "mode" => 6,
                _ => return Err(error("unknown YAML field")),
            };
            if fields[index].replace(value.trim()).is_some() {
                return Err(error("duplicate YAML field"));
            }
        }
        let required = |i: usize| fields[i].ok_or_else(|| error("missing YAML field"));
        let origin = required(2)?;
        let origin = origin
            .strip_prefix('[')
            .and_then(|s| s.strip_suffix(']'))
            .ok_or_else(|| error("origin must be [x, y, yaw]"))?;
        let mut values = origin.split(',');
        let mut xyz = [0.0; 3];
        for value in &mut xyz {
            *value = number(
                values
                    .next()
                    .ok_or_else(|| error("origin needs three values"))?,
            )?;
        }
        if values.next().is_some() {
            return Err(error("origin needs three values"));
        }
        let negate = match required(3)? {
            "0" | "false" => 0,
            "1" | "true" => 1,
            _ => return Err(error("invalid negate")),
        };
        let mode = if let Some(text) = fields[6] {
            let value = scalar(text)?;
            CuMapMode::deserialize(
                serde::de::value::StrDeserializer::<serde::de::value::Error>::new(&value),
            )
            .map_err(|_| error("only trinary mode is supported"))?
        } else {
            CuMapMode::default()
        };
        MetadataWire {
            image: scalar(required(0)?)?,
            resolution: number(required(1)?)?,
            origin: xyz,
            negate,
            occupied_thresh: number(required(4)?)?,
            free_thresh: number(required(5)?)?,
            mode,
        }
        .try_into()
    }
}

fn error(message: &str) -> CuError {
    CuError::from(message)
}
fn number(text: &str) -> CuResult<f64> {
    text.trim()
        .parse()
        .map_err(|_| error("invalid YAML number"))
}

// Locate comments without treating '#' inside quoted paths as a comment.
fn uncomment(line: &str) -> CuResult<&str> {
    // Only a quote at the beginning of a scalar opens a quoted scalar.
    // Apostrophes and quotes embedded in a plain filename remain literal.
    let value_start = line.find(':').map(|i| i + 1).unwrap_or(0);
    let value = &line[value_start..];
    let scalar_start = value_start + value.len() - value.trim_start().len();
    let mut quote = None;
    let mut escaped = false;
    let mut chars = line.char_indices().peekable();
    while let Some((i, c)) = chars.next() {
        if escaped {
            escaped = false;
            continue;
        }
        if quote == Some('"') && c == '\\' {
            escaped = true;
            continue;
        }
        if quote == Some(c) {
            if c == '\'' && chars.peek().is_some_and(|(_, next)| *next == '\'') {
                chars.next();
            } else {
                quote = None;
            }
        } else if i == scalar_start && (c == '\'' || c == '"') {
            quote = Some(c);
        }
        if c == '#' && quote.is_none() && (i == 0 || line.as_bytes()[i - 1].is_ascii_whitespace()) {
            return Ok(&line[..i]);
        }
    }
    if quote.is_some() {
        return Err(error("unterminated YAML quote"));
    }
    Ok(line)
}

fn scalar(text: &str) -> CuResult<String> {
    if let Some(s) = text.strip_prefix('\'').and_then(|s| s.strip_suffix('\'')) {
        let mut result = String::new();
        let mut chars = s.chars();
        while let Some(c) = chars.next() {
            if c == '\'' && chars.next() != Some('\'') {
                return Err(error("invalid YAML single quote"));
            }
            result.push(c);
        }
        return Ok(result);
    }
    if let Some(s) = text.strip_prefix('"').and_then(|s| s.strip_suffix('"')) {
        let mut result = String::new();
        let mut chars = s.chars();
        while let Some(c) = chars.next() {
            if c == '\\' {
                match chars.next() {
                    Some('"') => result.push('"'),
                    Some('\\') => result.push('\\'),
                    Some('/') => result.push('/'),
                    _ => return Err(error("unsupported YAML escape")),
                }
            } else if c == '"' {
                return Err(error("invalid YAML double quote"));
            } else {
                result.push(c);
            }
        }
        return Ok(result);
    }
    if text.is_empty()
        || text.starts_with(['!', '&', '*', '[', '{', '|', '>', '\'', '"'])
        || text.contains(": ")
    {
        return Err(error("unsupported YAML scalar"));
    }
    Ok(text.to_string())
}

struct Buffer<'a> {
    bytes: &'a mut [u8],
    len: usize,
}
impl Write for Buffer<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let end = self.len.checked_add(s.len()).ok_or(core::fmt::Error)?;
        self.bytes
            .get_mut(self.len..end)
            .ok_or(core::fmt::Error)?
            .copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// Encodes a binary P5 PGM in caller storage, returning bytes written.
/// Top image row is grid row `H - 1`; occupied/free/unknown are 0/254/205.
/// No allocation on success. The buffer may be changed on error.
pub fn encode_pgm<const W: usize, const H: usize>(
    grid: &CuOccupancyGrid<W, H>,
    output: &mut [u8],
) -> CuResult<usize> {
    let count = pixel_count::<W, H>()?;
    let mut buffer = Buffer {
        bytes: output,
        len: 0,
    };
    write!(buffer, "P5\n{W} {H}\n255\n").map_err(|_| error("PGM output buffer too small"))?;
    let end = buffer
        .len
        .checked_add(count)
        .ok_or_else(|| error("PGM size overflow"))?;
    let pixels = buffer
        .bytes
        .get_mut(buffer.len..end)
        .ok_or_else(|| error("PGM output buffer too small"))?;
    for row in 0..H {
        for x in 0..W {
            pixels[row * W + x] = match grid
                .get(CuCell2::new(x, H - 1 - row))
                .ok_or_else(|| error("grid cell unavailable"))?
            {
                CuOccupancy::Occupied => 0,
                CuOccupancy::Free => 254,
                CuOccupancy::Unknown => 205,
            };
        }
    }
    Ok(end)
}

fn pixel_count<const W: usize, const H: usize>() -> CuResult<usize> {
    if W == 0 || H == 0 {
        return Err(error("PGM dimensions must be positive"));
    }
    W.checked_mul(H).ok_or_else(|| error("PGM size overflow"))
}

fn token<'a>(input: &'a [u8], pos: &mut usize) -> CuResult<&'a [u8]> {
    loop {
        while input.get(*pos).is_some_and(u8::is_ascii_whitespace) {
            *pos += 1;
        }
        if input.get(*pos) != Some(&b'#') {
            break;
        }
        while input.get(*pos).is_some_and(|b| *b != b'\n') {
            *pos += 1;
        }
    }
    let start = *pos;
    while input
        .get(*pos)
        .is_some_and(|b| !b.is_ascii_whitespace() && *b != b'#')
    {
        *pos += 1;
    }
    if start == *pos {
        return Err(error("truncated PGM header"));
    }
    Ok(&input[start..*pos])
}
fn integer(bytes: &[u8]) -> CuResult<usize> {
    if !bytes.iter().all(u8::is_ascii_digit) {
        return Err(error("invalid PGM integer"));
    }
    core::str::from_utf8(bytes)
        .map_err(|_| error("invalid PGM header"))?
        .parse()
        .map_err(|_| error("invalid PGM integer"))
}

/// Decodes P5 bytes into an existing grid, checking image size and metadata geometry.
/// Supports 8-bit max values 1..=255, comments in the header and LF/CRLF headers.
/// Uses inclusive Nav2 thresholds (equality is occupied/free). No allocation on success.
/// Errors leave the grid unchanged.
pub fn decode_pgm<const W: usize, const H: usize>(
    input: &[u8],
    metadata: &CuMapMetadata,
    output: &mut CuOccupancyGrid<W, H>,
) -> CuResult<()> {
    metadata.validate()?;
    if metadata.origin.theta.get::<radian>() != 0.0
        || output.geometry().resolution != metadata.resolution
        || output.geometry().origin != metadata.origin.position()
    {
        return Err(error("map geometry does not match axis-aligned grid"));
    }
    let count = pixel_count::<W, H>()?;
    let mut pos = 0;
    if token(input, &mut pos)? != b"P5" {
        return Err(error("expected binary P5 PGM"));
    }
    let width = integer(token(input, &mut pos)?)?;
    let height = integer(token(input, &mut pos)?)?;
    let max = integer(token(input, &mut pos)?)?;
    if width != W || height != H {
        return Err(error("PGM dimensions do not match grid"));
    }
    if max == 0 || max > 255 {
        return Err(error("PGM max value must be 1..=255"));
    }
    // Consume exactly the raster separator: pixel bytes may themselves be whitespace or '#'.
    let separator = *input
        .get(pos)
        .ok_or_else(|| error("missing PGM raster separator"))?;
    if !separator.is_ascii_whitespace() {
        return Err(error("missing PGM raster separator"));
    }
    pos += 1;
    // CR then LF is a CRLF separator only if the raster is one byte too long;
    // otherwise the LF is the first pixel (value 10).
    if separator == b'\r' && input.get(pos) == Some(&b'\n') && input.len() - pos == count + 1 {
        pos += 1;
    }
    let pixels = input
        .get(pos..)
        .filter(|p| p.len() == count)
        .ok_or_else(|| error("PGM raster length mismatch"))?;
    if pixels.iter().any(|p| usize::from(*p) > max) {
        return Err(error("PGM pixel exceeds max value"));
    }
    for row in 0..H {
        for x in 0..W {
            // Subtract in integers before dividing so decimal thresholds compare exactly.
            let pixel = usize::from(pixels[row * W + x]);
            let level = if metadata.negate { pixel } else { max - pixel };
            let probability = level as f64 / max as f64;
            let state = if probability >= metadata.occupied_thresh {
                CuOccupancy::Occupied
            } else if probability <= metadata.free_thresh {
                CuOccupancy::Free
            } else {
                CuOccupancy::Unknown
            };
            output.set(CuCell2::new(x, H - 1 - row), state)?;
        }
    }
    Ok(())
}

/// Writes `path_stem.pgm` and `path_stem.yaml` with ROS trinary defaults.
/// Allocates and performs filesystem I/O. Existing files are overwritten; writes are not atomic.
///
/// # Example
///
/// ```no_run
/// use cu29::units::si::f64::Length;
/// use cu29::units::si::length::meter;
/// use meander_core::CuPoint2;
/// use meander_grid::{CuGridGeometry2, CuOccupancyGrid, CuOccupancyUpdateParams, load_map, save_map};
///
/// let geometry = CuGridGeometry2::new(Length::new::<meter>(0.05), CuPoint2::default())?;
/// let grid = CuOccupancyGrid::<64, 64>::from_params(geometry, CuOccupancyUpdateParams::default())?;
/// save_map(&grid, "warehouse")?;
/// let restored = load_map::<64, 64>("warehouse.yaml")?;
/// assert_eq!(restored.geometry(), grid.geometry());
/// # Ok::<(), cu29::prelude::CuError>(())
/// ```
#[cfg(feature = "std")]
pub fn save_map<const W: usize, const H: usize>(
    grid: &CuOccupancyGrid<W, H>,
    path_stem: impl AsRef<std::path::Path>,
) -> CuResult<()> {
    let stem = path_stem.as_ref();
    let mut image = stem.as_os_str().to_os_string();
    image.push(".pgm");
    let image = std::path::PathBuf::from(image);
    let mut yaml = stem.as_os_str().to_os_string();
    yaml.push(".yaml");
    let name = image
        .file_name()
        .and_then(|s| s.to_str())
        .ok_or_else(|| error("map filename must be UTF-8"))?;
    let metadata = CuMapMetadata::new(name.to_string(), grid.geometry())?;
    let text = metadata.to_yaml()?;
    let size = pixel_count::<W, H>()?
        .checked_add(128)
        .ok_or_else(|| error("PGM size overflow"))?;
    let mut bytes = alloc::vec![0; size];
    let len = encode_pgm(grid, &mut bytes)?;
    std::fs::write(image, &bytes[..len])
        .map_err(|e| CuError::new_with_cause("cannot write map image", e))?;
    std::fs::write(yaml, text).map_err(|e| CuError::new_with_cause("cannot write map YAML", e))
}

/// Loads a YAML sidecar and its relative or absolute P5 image path.
/// Image dimensions must equal `W`/`H`; resolution must be positive and finite.
/// The returned grid takes its resolution/origin from YAML; nonzero yaw errors.
#[cfg(feature = "std")]
pub fn load_map<const W: usize, const H: usize>(
    path_yaml: impl AsRef<std::path::Path>,
) -> CuResult<CuOccupancyGrid<W, H>> {
    let path = path_yaml.as_ref();
    let text = std::fs::read_to_string(path)
        .map_err(|e| CuError::new_with_cause("cannot read map YAML", e))?;
    let metadata = CuMapMetadata::from_yaml(&text)?;
    let image = path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .join(&metadata.image);
    let bytes =
        std::fs::read(image).map_err(|e| CuError::new_with_cause("cannot read map image", e))?;
    let mut grid = CuOccupancyGrid::from_params(
        CuGridGeometry2::new(metadata.resolution, metadata.origin.position())?,
        crate::CuOccupancyUpdateParams::default(),
    )?;
    decode_pgm(&bytes, &metadata, &mut grid)?;
    Ok(grid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metadata() -> CuMapMetadata {
        CuMapMetadata::from_yaml(include_str!("../tests/fixtures/nav2.yaml")).unwrap()
    }
    fn grid<const W: usize, const H: usize>(m: &CuMapMetadata) -> CuOccupancyGrid<W, H> {
        CuOccupancyGrid::from_params(
            CuGridGeometry2::new(m.resolution, m.origin.position()).unwrap(),
            crate::CuOccupancyUpdateParams::default(),
        )
        .unwrap()
    }

    #[test]
    fn test_round_trip_and_orientation() {
        let m = metadata();
        let mut input = grid::<3, 2>(&m);
        input.fill(CuOccupancy::Free);
        input
            .set(CuCell2::new(1, 0), CuOccupancy::Occupied)
            .unwrap();
        input.set(CuCell2::new(2, 1), CuOccupancy::Unknown).unwrap();
        let mut bytes = [0; 64];
        let len = encode_pgm(&input, &mut bytes).unwrap();
        assert_eq!(&bytes[..len], b"P5\n3 2\n255\n\xfe\xfe\xcd\xfe\x00\xfe");
        let mut output = grid::<3, 2>(&m);
        decode_pgm(&bytes[..len], &m, &mut output).unwrap();
        for y in 0..2 {
            for x in 0..3 {
                assert_eq!(
                    input.get(CuCell2::new(x, y)),
                    output.get(CuCell2::new(x, y))
                );
            }
        }
        assert_eq!(CuMapMetadata::from_yaml(&m.to_yaml().unwrap()).unwrap(), m);
    }

    #[test]
    fn test_nav2_fixture() {
        let m = metadata();
        let mut output = grid::<3, 2>(&m);
        decode_pgm(
            include_bytes!("../tests/fixtures/nav2.pgm"),
            &m,
            &mut output,
        )
        .unwrap();
        assert_eq!(output.get(CuCell2::new(0, 1)), Some(CuOccupancy::Occupied));
        assert_eq!(output.get(CuCell2::new(1, 1)), Some(CuOccupancy::Free));
        assert_eq!(output.get(CuCell2::new(2, 1)), Some(CuOccupancy::Unknown));
        assert_eq!(output.get(CuCell2::new(2, 0)), Some(CuOccupancy::Occupied));
    }

    #[test]
    fn test_thresholds_negate_and_equality() {
        let mut m = metadata();
        m.free_thresh = 0.25;
        m.occupied_thresh = 0.75;
        // max=100 makes threshold equality exactly representable.
        let bytes = b"P5\n5 1\n100\n\x00\x19\x32\x4b\x64";
        let mut output = grid::<5, 1>(&m);
        for negate in [false, true] {
            m.negate = negate;
            decode_pgm(bytes, &m, &mut output).unwrap();
            let expected = if negate {
                [
                    CuOccupancy::Free,
                    CuOccupancy::Free,
                    CuOccupancy::Unknown,
                    CuOccupancy::Occupied,
                    CuOccupancy::Occupied,
                ]
            } else {
                [
                    CuOccupancy::Occupied,
                    CuOccupancy::Occupied,
                    CuOccupancy::Unknown,
                    CuOccupancy::Free,
                    CuOccupancy::Free,
                ]
            };
            for (x, state) in expected.into_iter().enumerate() {
                assert_eq!(output.get(CuCell2::new(x, 0)), Some(state));
            }
        }
    }

    #[test]
    fn test_malformed_inputs_and_geometry_leave_output_unchanged() {
        let m = metadata();
        let mut output = grid::<3, 2>(&m);
        output.fill(CuOccupancy::Free);
        for bytes in [
            b"P5\n2 3\n255\n\0\0\0\0\0\0".as_slice(),
            b"P5\n3 2\n255\n\0",
            b"P2\n3 2\n255\n000000",
            b"P5\n3 2\n256\n000000",
            b"P5\n3 2\n1\n000000",
        ] {
            assert!(decode_pgm(bytes, &m, &mut output).is_err());
            assert_eq!(output.get(CuCell2::new(0, 0)), Some(CuOccupancy::Free));
        }
        let mut wrong = m.clone();
        wrong.resolution = Length::new::<meter>(0.1);
        assert!(
            decode_pgm(
                include_bytes!("../tests/fixtures/nav2.pgm"),
                &wrong,
                &mut output
            )
            .is_err()
        );
        wrong = m.clone();
        wrong.origin.theta = Angle::new::<radian>(0.2);
        assert!(
            decode_pgm(
                include_bytes!("../tests/fixtures/nav2.pgm"),
                &wrong,
                &mut output
            )
            .is_err()
        );
        assert!(encode_pgm(&output, &mut [0; 5]).is_err());
        assert!(encode_pgm(&grid::<0, 2>(&m), &mut [0; 64]).is_err());
    }

    #[test]
    fn test_raster_whitespace_and_hash_are_pixels() {
        let mut m = metadata();
        m.negate = true;
        let mut output = grid::<3, 1>(&m);
        decode_pgm(b"P5\r\n# comment\r\n3 1\r\n255\r\n\n #", &m, &mut output).unwrap();
        for x in 0..3 {
            assert_eq!(output.get(CuCell2::new(x, 0)), Some(CuOccupancy::Free));
        }
    }

    #[test]
    fn test_cr_separator_followed_by_lf_pixel() {
        let mut m = metadata();
        m.negate = true;
        let mut output = grid::<1, 1>(&m);
        // A lone CR separator, then one pixel of value 10 (LF): not a CRLF.
        decode_pgm(b"P5\n1 1\n255\r\n", &m, &mut output).unwrap();
        assert_eq!(output.get(CuCell2::new(0, 0)), Some(CuOccupancy::Free));
    }

    #[test]
    fn test_decimal_thresholds_compare_exactly() {
        let mut m = metadata();
        m.negate = false;
        m.free_thresh = 0.3;
        m.occupied_thresh = 0.9;
        let mut output = grid::<2, 1>(&m);
        // max=100: pixel 70 is p=0.3 (Free at equality), pixel 10 is p=0.9 (Occupied).
        decode_pgm(b"P5\n2 1\n100\n\x46\x0a", &m, &mut output).unwrap();
        assert_eq!(output.get(CuCell2::new(0, 0)), Some(CuOccupancy::Free));
        assert_eq!(output.get(CuCell2::new(1, 0)), Some(CuOccupancy::Occupied));
    }

    #[test]
    fn test_metadata_serde_uses_nav2_scalar_schema() {
        let m = metadata();
        let text = serde_json::to_string(&m).unwrap();
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(value["resolution"], 0.05);
        assert_eq!(value["origin"], serde_json::json!([-1.0, -2.0, 0.0]));
        assert_eq!(value["negate"], 0);
        assert_eq!(value["mode"], "trinary");
        assert_eq!(serde_json::from_str::<CuMapMetadata>(&text).unwrap(), m);
        let mut invalid = value.clone();
        invalid["extra"] = serde_json::json!(1);
        assert!(serde_json::from_value::<CuMapMetadata>(invalid).is_err());
        let mut invalid = value.clone();
        invalid["resolution"] = serde_json::json!(-0.1);
        assert!(serde_json::from_value::<CuMapMetadata>(invalid).is_err());
        let mut invalid = value;
        invalid["negate"] = serde_json::json!(2);
        assert!(serde_json::from_value::<CuMapMetadata>(invalid).is_err());
    }

    #[test]
    fn test_yaml_validation_and_quoted_paths() {
        let mut m = metadata();
        for path in ["room #1's map.pgm", "a\\b.pgm", "a: b.pgm", "café.pgm"] {
            m.image = path.to_string();
            assert_eq!(CuMapMetadata::from_yaml(&m.to_yaml().unwrap()).unwrap(), m);
        }
        let fixture = include_str!("../tests/fixtures/nav2.yaml");
        assert_eq!(
            CuMapMetadata::from_yaml(&fixture.replace("nav2.pgm", "robot's map.pgm"))
                .unwrap()
                .image,
            "robot's map.pgm"
        );
        assert_eq!(
            CuMapMetadata::from_yaml(&format!("---\n{fixture}...\n")).unwrap(),
            metadata()
        );
        assert_eq!(
            CuMapMetadata::from_yaml(&fixture.replace("mode: trinary\n", "")).unwrap(),
            metadata()
        );
        assert_eq!(
            CuMapMetadata::from_yaml(&fixture.replace("nav2.pgm", "\"room #1.pgm\" # comment"))
                .unwrap()
                .image,
            "room #1.pgm"
        );
        for text in [
            format!("{fixture}\nimage: duplicate.pgm"),
            format!("{fixture}\nunknown: 1"),
            format!("{fixture}\n---"),
            format!("{fixture}\n...\nimage: extra.pgm"),
            fixture.replace("trinary", "scale"),
            fixture.replace("0.05", "NaN"),
            fixture.replace("0.65", "0.1"),
            fixture.replace("negate: 0", "negate: 2"),
            fixture.replace("[-1.0, -2.0, 0.0]", "[1, 2]"),
            fixture.replace("nav2.pgm", "'unterminated"),
        ] {
            assert!(CuMapMetadata::from_yaml(&text).is_err(), "{text}");
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_filesystem_round_trip_and_mismatched_size() {
        let dir = tempfile::tempdir().unwrap();
        let stem = dir.path().join("room.v1's #map");
        let m = metadata();
        let mut input = grid::<3, 2>(&m);
        decode_pgm(include_bytes!("../tests/fixtures/nav2.pgm"), &m, &mut input).unwrap();
        save_map(&input, &stem).unwrap();
        let yaml = dir.path().join("room.v1's #map.yaml");
        let output = load_map::<3, 2>(&yaml).unwrap();
        assert_eq!(input.geometry(), output.geometry());
        for y in 0..2 {
            for x in 0..3 {
                assert_eq!(
                    input.get(CuCell2::new(x, y)),
                    output.get(CuCell2::new(x, y))
                );
            }
        }
        assert!(load_map::<2, 3>(&yaml).is_err());
    }
}
