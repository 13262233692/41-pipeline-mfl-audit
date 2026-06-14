use std::io::Write;
use std::path::PathBuf;

fn create_test_mfl(num_channels: u16, num_axes: u8, num_frames: usize) -> PathBuf {
    let dir = std::env::temp_dir().join("mfl_audit_test");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(format!("test_{}_{}_{}.mfl", num_channels, num_axes, num_frames));

    let mut buf = Vec::new();

    let mut hdr = [0u8; 64];
    hdr[0..4].copy_from_slice(&[0x4D, 0x46, 0x4C, 0x31]);
    hdr[4..6].copy_from_slice(&1u16.to_le_bytes());
    hdr[6..8].copy_from_slice(&num_channels.to_le_bytes());
    hdr[8] = num_axes;
    hdr[9] = 24;
    hdr[10..14].copy_from_slice(&1000u32.to_le_bytes());
    hdr[14..18].copy_from_slice(&508.0f32.to_le_bytes());
    hdr[18..22].copy_from_slice(&7.1f32.to_le_bytes());
    hdr[22..26].copy_from_slice(&1.0f32.to_le_bytes());
    hdr[26..34].copy_from_slice(&1700000000u64.to_le_bytes());
    buf.extend_from_slice(&hdr);

    let sample_bytes = 3usize;
    let frame_data = num_channels as usize * num_axes as usize * sample_bytes;

    for i in 0..num_frames {
        let mut fhdr = [0u8; 16];
        fhdr[0..4].copy_from_slice(&(i as u32).to_le_bytes());
        fhdr[4..8].copy_from_slice(&((i as u32) * 1000).to_le_bytes());
        let dist = i as f32 * 0.5;
        fhdr[8..12].copy_from_slice(&dist.to_le_bytes());
        fhdr[12..16].copy_from_slice(&500.0f32.to_le_bytes());
        buf.extend_from_slice(&fhdr);

        let mut data = vec![0u8; frame_data];
        for ch in 0..num_channels as usize {
            for ax in 0..num_axes as usize {
                let off = ch * (num_axes as usize) * sample_bytes + ax * sample_bytes;
                let val = ((ch as i32 + ax as i32 + i as i32) % 256) as i32;
                let signed = if val > 127 { val - 256 } else { val };
                let unsigned = signed as i32 & 0xFFFFFF;
                data[off] = (unsigned & 0xFF) as u8;
                data[off + 1] = ((unsigned >> 8) & 0xFF) as u8;
                data[off + 2] = ((unsigned >> 16) & 0xFF) as u8;
            }
        }
        buf.extend_from_slice(&data);
    }

    let mut f = std::fs::File::create(&path).unwrap();
    f.write_all(&buf).unwrap();
    path
}

fn create_defect_mfl() -> PathBuf {
    let num_channels: u16 = 32;
    let num_axes: u8 = 1;
    let num_frames = 200;
    let dir = std::env::temp_dir().join("mfl_audit_test");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("defect_test.mfl");

    let mut buf = Vec::new();

    let mut hdr = [0u8; 64];
    hdr[0..4].copy_from_slice(&[0x4D, 0x46, 0x4C, 0x31]);
    hdr[4..6].copy_from_slice(&1u16.to_le_bytes());
    hdr[6..8].copy_from_slice(&num_channels.to_le_bytes());
    hdr[8] = num_axes;
    hdr[9] = 24;
    hdr[10..14].copy_from_slice(&1000u32.to_le_bytes());
    hdr[14..18].copy_from_slice(&508.0f32.to_le_bytes());
    hdr[18..22].copy_from_slice(&7.1f32.to_le_bytes());
    hdr[22..26].copy_from_slice(&1.0f32.to_le_bytes());
    hdr[26..34].copy_from_slice(&1700000000u64.to_le_bytes());
    buf.extend_from_slice(&hdr);

    let sample_bytes = 3usize;
    let frame_data = num_channels as usize * num_axes as usize * sample_bytes;

    let defect_center_row = 100usize;
    let defect_center_col = 16usize;
    let defect_radius = 8usize;

    for i in 0..num_frames {
        let mut fhdr = [0u8; 16];
        fhdr[0..4].copy_from_slice(&(i as u32).to_le_bytes());
        fhdr[4..8].copy_from_slice(&((i as u32) * 1000).to_le_bytes());
        let dist = i as f32 * 0.5;
        fhdr[8..12].copy_from_slice(&dist.to_le_bytes());
        fhdr[12..16].copy_from_slice(&500.0f32.to_le_bytes());
        buf.extend_from_slice(&fhdr);

        let mut data = vec![0u8; frame_data];
        for ch in 0..num_channels as usize {
            let dr = (i as isize - defect_center_row as isize).abs() as usize;
            let dc = (ch as isize - defect_center_col as isize).abs() as usize;
            let is_defect = dr < defect_radius && dc < defect_radius;
            let base_val = 10i32;
            let val = if is_defect {
                let dist_sq = (dr * dr + dc * dc) as f32;
                let r_sq = (defect_radius * defect_radius) as f32;
                let strength = 200.0 * (1.0 - dist_sq / r_sq).max(0.0);
                base_val + strength as i32
            } else {
                base_val
            };
            let unsigned = val & 0xFFFFFF;
            let off = ch * sample_bytes;
            data[off] = (unsigned & 0xFF) as u8;
            data[off + 1] = ((unsigned >> 8) & 0xFF) as u8;
            data[off + 2] = ((unsigned >> 16) & 0xFF) as u8;
        }
        buf.extend_from_slice(&data);
    }

    let mut f = std::fs::File::create(&path).unwrap();
    f.write_all(&buf).unwrap();
    path
}

#[test]
fn test_file_header_parse() {
    let path = create_test_mfl(16, 3, 10);
    let stream = mfl_format_stream_open(&path);
    let hdr = stream.header();
    assert_eq!(hdr.magic, [0x4D, 0x46, 0x4C, 0x31]);
    assert_eq!(hdr.version, 1);
    assert_eq!(hdr.num_channels, 16);
    assert_eq!(hdr.num_axes, 3);
    assert_eq!(hdr.sample_resolution_bits, 24);
    assert_eq!(hdr.frame_rate_hz, 1000);
}

#[test]
fn test_frame_count() {
    let path = create_test_mfl(8, 2, 50);
    let stream = mfl_format_stream_open(&path);
    assert_eq!(stream.num_frames(), 50);
}

#[test]
fn test_frame_header_parse() {
    let path = create_test_mfl(4, 1, 5);
    let stream = mfl_format_stream_open(&path);
    let f0 = stream.frame(0).unwrap();
    assert_eq!(f0.hdr.frame_index, 0);
    assert_eq!(f0.hdr.timestamp_us, 0);
    let f4 = stream.frame(4).unwrap();
    assert_eq!(f4.hdr.frame_index, 4);
    assert_eq!(f4.hdr.timestamp_us, 4000);
}

#[test]
fn test_decode_channel_axes() {
    let path = create_test_mfl(4, 2, 3);
    let stream = mfl_format_stream_open(&path);
    let frame = stream.frame(0).unwrap();
    let axes = stream.decode_channel_axes(&frame, 0);
    assert_eq!(axes.len(), 2);
}

#[test]
fn test_grid_build() {
    let path = create_test_mfl(8, 1, 20);
    let stream = mfl_format_stream_open(&path);
    let grid = mfl_build_grid(&stream, 0, 500.0, 5, 3.0);
    assert_eq!(grid.cols, 8);
    assert!(grid.rows > 0);
}

#[test]
fn test_hough_no_defects_flat() {
    let path = create_test_mfl(8, 1, 20);
    let stream = mfl_format_stream_open(&path);
    let grid = mfl_build_grid(&stream, 0, 500.0, 5, 3.0);
    let mut detector = mfl_audit::HoughDetector::new();
    detector.edge_threshold = 50.0;
    detector.vote_threshold = 20;
    let defects = detector.detect(&grid);
    assert!(defects.len() <= 10, "Flat data should produce few or no defects, got {}", defects.len());
}

#[test]
fn test_defect_detection() {
    let path = create_defect_mfl();
    let stream = mfl_format_stream_open(&path);
    let grid = mfl_build_grid(&stream, 0, 500.0, 5, 0.0);
    let mut detector = mfl_audit::HoughDetector::new();
    detector.edge_threshold = 5.0;
    detector.vote_threshold = 3;
    let defects = detector.detect(&grid);
    assert!(!defects.is_empty(), "Should detect at least one defect region in synthetic data");
}

#[test]
fn test_grid_normalize() {
    let path = create_test_mfl(4, 1, 10);
    let stream = mfl_format_stream_open(&path);
    let grid = mfl_build_grid(&stream, 0, 500.0, 3, 0.0);
    let u8buf = grid.normalize_to_u8();
    assert_eq!(u8buf.len(), grid.rows * grid.cols);
    let has_nonzero = u8buf.iter().any(|&v| v > 0);
    assert!(has_nonzero);
}

fn mfl_format_stream_open(path: &PathBuf) -> mfl_audit::MflStream {
    mfl_audit::MflStream::open(path).unwrap()
}

fn mfl_build_grid(
    stream: &mfl_audit::MflStream,
    axis: u8,
    vel: f32,
    win: usize,
    sigma: f32,
) -> mfl_audit::GridMatrix {
    mfl_audit::build_grid(stream, axis, vel, win, sigma)
}
