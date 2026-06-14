use crate::dipole::Vec3;
use crate::hough::DefectRegion;
use crate::signal_proc::{full_preprocess, VelocityCorrector};
use crate::stream_parser::MflStream;

#[derive(Debug, Clone)]
pub struct DefectSpatialCube {
    pub region_id: usize,
    pub center_row: usize,
    pub center_col: usize,
    pub row_window: std::ops::Range<usize>,
    pub col_window: std::ops::Range<usize>,
    pub axial: Vec<Vec<f64>>,
    pub transverse: Vec<Vec<f64>>,
    pub radial: Vec<Vec<f64>>,
    pub distance_mm: Vec<f64>,
    pub angle_deg: Vec<f64>,
    pub amplitude_peak: f64,
}

impl DefectSpatialCube {
    pub fn rows(&self) -> usize { self.axial.len() }
    pub fn cols(&self) -> usize { self.axial.first().map(|r| r.len()).unwrap_or(0) }

    pub fn field_at(&self, r: usize, c: usize) -> Vec3 {
        let ax = self.axial.get(r).and_then(|row| row.get(c)).copied().unwrap_or(0.0);
        let tr = self.transverse.get(r).and_then(|row| row.get(c)).copied().unwrap_or(0.0);
        let rd = self.radial.get(r).and_then(|row| row.get(c)).copied().unwrap_or(0.0);
        Vec3::new(ax, tr, rd)
    }

    pub fn sensor_list(&self) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(self.rows() * self.cols());
        for r in 0..self.rows() {
            for c in 0..self.cols() {
                let d = self.distance_mm.get(r).copied().unwrap_or(0.0);
                let a = self.angle_deg.get(c).copied().unwrap_or(0.0);
                out.push((d, a));
            }
        }
        out
    }

    pub fn measured_field_vec(&self) -> Vec<Vec3> {
        let mut out = Vec::with_capacity(self.rows() * self.cols());
        for r in 0..self.rows() {
            for c in 0..self.cols() {
                out.push(self.field_at(r, c));
            }
        }
        out
    }
}

pub fn extract_channel_axis_vec(
    stream: &MflStream,
    channel: u16,
    axis: u8,
    nominal_vel: f32,
    med_window: usize,
    sigma_thresh: f32,
) -> Vec<f32> {
    full_preprocess(stream, channel, axis, nominal_vel, med_window, sigma_thresh)
}

fn extract_axis_for_all_channels(
    stream: &MflStream,
    axis: u8,
    nominal_vel: f32,
    _med_window: usize,
    _sigma_thresh: f32,
    n_channels: usize,
    n_frames: usize,
) -> Vec<Vec<f32>> {
    let vc = VelocityCorrector::new(nominal_vel);
    (0..n_channels)
        .into_iter()
        .map(|ch| {
            let buf = vc.load_channel_from_owned(stream, ch as u16, axis);
            let mut vec = vc.correct_owned(&buf);
            vec.truncate(n_frames);
            vec.resize(n_frames, 0.0);
            vec
        })
        .collect()
}

pub fn extract_defect_cubes(
    stream: &MflStream,
    regions: &[DefectRegion],
    nominal_vel: f32,
    med_window: usize,
    sigma_thresh: f32,
    _wall_thickness_mm: f32,
) -> Vec<DefectSpatialCube> {
    let n_frames = stream.num_frames();
    let n_channels = stream.header().num_channels as usize;
    let n_axes = stream.header().num_axes as usize;
    if n_frames == 0 || n_channels == 0 {
        return Vec::new();
    }

    let mut per_axis: Vec<Vec<Vec<f32>>> = Vec::with_capacity(n_axes.max(1));
    for axis_idx in 0..n_axes.max(1) {
        let axis = if n_axes >= 3 { axis_idx } else { 0 } as u8;
        per_axis.push(extract_axis_for_all_channels(
            stream, axis, nominal_vel, med_window, sigma_thresh, n_channels, n_frames,
        ));
    }
    while per_axis.len() < 3 {
        per_axis.push(per_axis[0].iter().map(|c| vec![0.0f32; c.len()]).collect());
    }

    let distances: Vec<f64> = (0..n_frames)
        .map(|f| {
            stream.frame(f)
                .map(|fr| fr.hdr.distance_mm as f64)
                .unwrap_or(f as f64 * 0.5)
        })
        .collect();

    let spacing_deg = stream.header().sensor_spacing_deg as f64;
    let angles: Vec<f64> = (0..n_channels)
        .map(|c| c as f64 * spacing_deg.max(1.0))
        .collect();

    let mut cubes = Vec::with_capacity(regions.len());
    for (rid, region) in regions.iter().enumerate() {
        let rr = region.radius_rows.max(5);
        let rc = region.radius_cols.max(3);

        let r_start = region.center_row.saturating_sub(rr).min(n_frames.saturating_sub(1));
        let r_end = (region.center_row + rr).min(n_frames);
        let c_start = region.center_col.saturating_sub(rc).min(n_channels.saturating_sub(1));
        let c_end = (region.center_col + rc).min(n_channels);

        if r_end <= r_start || c_end <= c_start {
            continue;
        }

        let nr = r_end - r_start;
        let nc = c_end - c_start;

        let mut axial = vec![vec![0.0f64; nc]; nr];
        let mut transverse = vec![vec![0.0f64; nc]; nr];
        let mut radial = vec![vec![0.0f64; nc]; nr];
        let mut peak = 0.0f64;

        for ri in 0..nr {
            for ci in 0..nc {
                let gr = r_start + ri;
                let gc = c_start + ci;
                let ax = per_axis[0].get(gc).and_then(|c| c.get(gr)).copied().unwrap_or(0.0) as f64;
                let tr = per_axis[1].get(gc).and_then(|c| c.get(gr)).copied().unwrap_or(0.0) as f64;
                let rd = per_axis[2].get(gc).and_then(|c| c.get(gr)).copied().unwrap_or(0.0) as f64;
                let mag = (ax * ax + tr * tr + rd * rd).sqrt();
                if mag > peak { peak = mag; }
                axial[ri][ci] = ax;
                transverse[ri][ci] = tr;
                radial[ri][ci] = rd;
            }
        }

        let d_slice = distances[r_start..r_end].to_vec();
        let a_slice = angles[c_start..c_end].to_vec();

        cubes.push(DefectSpatialCube {
            region_id: rid,
            center_row: region.center_row,
            center_col: region.center_col,
            row_window: r_start..r_end,
            col_window: c_start..c_end,
            axial,
            transverse,
            radial,
            distance_mm: d_slice,
            angle_deg: a_slice,
            amplitude_peak: peak,
        });
    }
    cubes
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::path::PathBuf;

    fn make_mfl(path: &PathBuf, channels: u16, frames: usize, center: (usize, usize), r: usize) {
        let mut buf = Vec::new();
        let mut hdr = [0u8; 64];
        hdr[0..4].copy_from_slice(&[0x4D, 0x46, 0x4C, 0x31]);
        hdr[4..6].copy_from_slice(&1u16.to_le_bytes());
        hdr[6..8].copy_from_slice(&channels.to_le_bytes());
        hdr[8] = 3;
        hdr[9] = 24;
        hdr[10..14].copy_from_slice(&1000u32.to_le_bytes());
        hdr[14..18].copy_from_slice(&508.0f32.to_le_bytes());
        hdr[18..22].copy_from_slice(&7.1f32.to_le_bytes());
        hdr[22..26].copy_from_slice(&1.0f32.to_le_bytes());
        buf.extend_from_slice(&hdr);
        let naxes = 3usize;
        let sb = 3usize;
        let fd = channels as usize * naxes * sb;

        for i in 0..frames {
            let mut fhdr = [0u8; 16];
            fhdr[0..4].copy_from_slice(&(i as u32).to_le_bytes());
            fhdr[8..12].copy_from_slice(&(i as f32 * 0.5).to_le_bytes());
            buf.extend_from_slice(&fhdr);
            let mut data = vec![0u8; fd];
            for c in 0..channels as usize {
                let in_defect = ((i as isize - center.0 as isize).unsigned_abs() as usize) < r
                    && ((c as isize - center.1 as isize).unsigned_abs() as usize) < r;
                let v = if in_defect { 200i32 } else { 10i32 };
                for a in 0..naxes {
                    let off = (c * naxes + a) * sb;
                    let u = (v + a as i32 * 5) & 0xFFFFFF;
                    data[off] = u as u8;
                    data[off + 1] = (u >> 8) as u8;
                    data[off + 2] = (u >> 16) as u8;
                }
            }
            buf.extend_from_slice(&data);
        }
        let mut f = std::fs::File::create(path).unwrap();
        f.write_all(&buf).unwrap();
    }

    #[test]
    fn extract_defect_window_dims() {
        let dir = std::env::temp_dir().join("mfl_defect_extract");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("test_extract.mfl");
        let (ch, fr, cr, cc, rr) = (16u16, 100usize, 50usize, 8usize, 10usize);
        make_mfl(&path, ch, fr, (cr, cc), rr);

        let stream = crate::stream_parser::MflStream::open(&path).unwrap();
        let grid = crate::grid::build_grid(&stream, 0, 500.0, 5, 0.0);
        let mut hd = crate::hough::HoughDetector::new();
        hd.edge_threshold = 0.1;
        hd.vote_threshold = 2;
        hd.nms_window = 3;
        let regions = hd.detect(&grid);

        let cubes = extract_defect_cubes(&stream, &regions, 500.0, 5, 0.0, 7.1);
        assert!(!cubes.is_empty() || regions.is_empty(), "Should produce cubes for regions");
        for cube in &cubes {
            assert_eq!(cube.rows(), cube.axial.len());
            assert!(cube.rows() > 0 && cube.cols() > 0);
            assert!(cube.amplitude_peak > 0.0);
        }
    }
}
