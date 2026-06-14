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
    let num_channels: u16 = 64;
    let num_axes: u8 = 1;
    let num_frames = 500;
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

    let cx = 250.0f32;
    let cy = 32.0f32;
    let a = 60.0f32;
    let b = 15.0f32;

    for i in 0..num_frames {
        let mut fhdr = [0u8; 16];
        fhdr[0..4].copy_from_slice(&(i as u32).to_le_bytes());
        fhdr[4..8].copy_from_slice(&((i as u32) * 1000).to_le_bytes());
        let dist = i as f32 * 0.5;
        fhdr[8..12].copy_from_slice(&dist.to_le_bytes());
        fhdr[12..16].copy_from_slice(&500.0f32.to_le_bytes());
        buf.extend_from_slice(&fhdr);

        let mut data = vec![0u8; frame_data];
        let rf = i as f32;
        for ch in 0..num_channels as usize {
            let cf = ch as f32;
            let dx = (rf - cx) / a;
            let dy = (cf - cy) / b;
            let ellipse = dx * dx + dy * dy;
            let base: i32 = 20;
            let val = if ellipse <= 1.0 {
                let edge_factor = 1.0 - ellipse.sqrt();
                base + (edge_factor * 200.0) as i32
            } else {
                base + ((i * 7 + ch * 3) % 5) as i32
            };
            let val = val.clamp(0, 255);
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
    let grid = mfl_build_grid(&stream, 0, 500.0, 3, 0.0);
    let mut detector = mfl_audit::HoughDetector::new();
    detector.edge_threshold = 0.5;
    detector.vote_threshold = 2;
    detector.nms_window = 3;
    let defects = detector.detect(&grid);
    assert!(!defects.is_empty(), "Should detect at least one defect region in synthetic data, got 0. Grid rows={} cols={}", grid.rows, grid.cols);
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

fn create_high_density_velocity_jump_mfl(
    num_channels: u16,
    num_frames: usize,
    weld_interval: usize,
) -> PathBuf {
    let dir = std::env::temp_dir().join("mfl_audit_test");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(format!("density_jump_ch{}_f{}.mfl", num_channels, num_frames));

    let num_axes: u8 = 1;
    let mut buf = Vec::new();

    let mut hdr = [0u8; 64];
    hdr[0..4].copy_from_slice(&[0x4D, 0x46, 0x4C, 0x31]);
    hdr[4..6].copy_from_slice(&1u16.to_le_bytes());
    hdr[6..8].copy_from_slice(&num_channels.to_le_bytes());
    hdr[8] = num_axes;
    hdr[9] = 24;
    hdr[10..14].copy_from_slice(&2000u32.to_le_bytes());
    hdr[14..18].copy_from_slice(&508.0f32.to_le_bytes());
    hdr[18..22].copy_from_slice(&7.1f32.to_le_bytes());
    hdr[22..26].copy_from_slice(&0.5f32.to_le_bytes());
    hdr[26..34].copy_from_slice(&1700000000u64.to_le_bytes());
    buf.extend_from_slice(&hdr);

    let sample_bytes = 3usize;
    let frame_data = num_channels as usize * num_axes as usize * sample_bytes;

    for i in 0..num_frames {
        let mut fhdr = [0u8; 16];
        fhdr[0..4].copy_from_slice(&(i as u32).to_le_bytes());
        fhdr[4..8].copy_from_slice(&((i as u32) * 500).to_le_bytes());

        let base_dist = i as f32 * 0.5;
        let at_weld = i > 0 && i % weld_interval == 0;
        let dist = if at_weld {
            base_dist + 500.0
        } else {
            base_dist
        };
        fhdr[8..12].copy_from_slice(&dist.to_le_bytes());

        let vel = if at_weld { 2000.0 } else { 500.0f32 };
        fhdr[12..16].copy_from_slice(&vel.to_le_bytes());
        buf.extend_from_slice(&fhdr);

        let mut data = vec![0u8; frame_data];
        for ch in 0..num_channels as usize {
            let signal = if at_weld {
                ((i + ch) % 200) as i32 - 100
            } else {
                ((i + ch * 3) % 50) as i32
            };
            let signal = signal.clamp(-128, 127);
            let unsigned = signal & 0xFFFFFF;
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
fn stress_128ch_velocity_weld_jumps() {
    let path = create_high_density_velocity_jump_mfl(128, 2000, 50);
    let stream = mfl_format_stream_open(&path);
    assert_eq!(stream.header().num_channels, 128);
    assert_eq!(stream.num_frames(), 2000);

    let grid = mfl_build_grid(&stream, 0, 500.0, 21, 3.0);
    assert_eq!(grid.cols, 128);
    assert_eq!(grid.rows, 2000);

    for c in 0..128 {
        let sample = grid.get(100, c);
        assert!(!sample.is_nan(), "NaN at column {}", c);
    }

    let mut detector = mfl_audit::HoughDetector::new();
    detector.edge_threshold = 10.0;
    detector.vote_threshold = 10;
    let _defects = detector.detect(&grid);
}

#[test]
fn stress_velocity_spiked_no_out_of_bounds() {
    let path = create_high_density_velocity_jump_mfl(64, 1000, 20);
    let stream = mfl_format_stream_open(&path);

    for c in 0..8 {
        use mfl_audit::signal_proc::VelocityCorrector;
        let vc = VelocityCorrector::new(500.0);
        let corrected = vc.correct_channel(&stream, c, 0);
        assert!(
            corrected.len() <= 1000 * 3,
            "Corrected overflow: {} for channel {}",
            corrected.len(),
            c
        );
        assert!(!corrected.is_empty());
        assert!(corrected.iter().all(|v| !v.is_nan()));
        assert!(corrected.iter().all(|v| !v.is_infinite()));
    }
}

#[test]
fn ringbuf_mpmc_high_contention() {
    use mfl_audit::ringbuf::RingBuffer;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;
    use std::thread;

    const PRODUCERS: u32 = 8;
    const CONSUMERS: u32 = 4;
    const OPS_PER_PRODUCER: u32 = 100_000;

    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
    struct Msg {
        prod: u32,
        seq: u32,
    }

    let rb = Arc::new(RingBuffer::<Msg>::new(1024));
    let produced = Arc::new(AtomicU64::new(0));
    let consumed = Arc::new(AtomicU64::new(0));
    let stop = Arc::new(AtomicU64::new(0));
    let total = PRODUCERS * OPS_PER_PRODUCER;

    let mut handles = Vec::new();

    for p in 0..PRODUCERS {
        let rb = rb.clone();
        let prod = produced.clone();
        handles.push(thread::spawn(move || {
            for s in 0..OPS_PER_PRODUCER {
                let msg = Msg { prod: p, seq: s };
                while !rb.enqueue(msg) {
                    std::hint::spin_loop();
                }
                prod.fetch_add(1, Ordering::Relaxed);
            }
        }));
    }

    let seen = Arc::new(std::sync::Mutex::new(std::collections::HashSet::with_capacity(
        total as usize,
    )));

    for _ in 0..CONSUMERS {
        let rb = rb.clone();
        let cons = consumed.clone();
        let stop = stop.clone();
        let seen = seen.clone();
        handles.push(thread::spawn(move || loop {
            if let Some(msg) = rb.dequeue() {
                let key = (msg.prod as u64) << 32 | (msg.seq as u64);
                assert!(seen.lock().unwrap().insert(key), "Duplicate message");
                let c = cons.fetch_add(1, Ordering::Relaxed) + 1;
                if c >= total as u64 {
                    stop.store(1, Ordering::Release);
                }
            } else if stop.load(Ordering::Acquire) == 1 {
                break;
            } else {
                std::hint::spin_loop();
            }
        }));
    }

    for h in handles {
        h.join().unwrap();
    }

    assert_eq!(
        seen.lock().unwrap().len() as u64,
        total as u64,
        "Missing/duplicate messages under high contention"
    );
}

#[test]
fn column_isolated_writes_no_aliasing() {
    use rayon::prelude::*;
    let rows = 1000usize;
    let cols = 128usize;
    let mut grid = mfl_audit::GridMatrix::new(rows, cols);

    let col_data: Vec<(usize, Box<[f32]>)> = (0..cols)
        .into_par_iter()
        .map(|c| {
            let mut col: Box<[f32]> = vec![0.0f32; rows].into_boxed_slice();
            for r in 0..rows {
                col[r] = ((c * rows + r) as f32) * 0.001;
            }
            (c, col)
        })
        .collect();

    for (c, data) in col_data {
        grid.install_column(c, data);
    }

    for c in 0..cols {
        for r in 0..rows {
            let v = grid.get(r, c);
            let expected = ((c * rows + r) as f32) * 0.001;
            assert!(
                (v - expected).abs() < 1e-6,
                "Mismatch at r={} c={}: got {} expected {}",
                r, c, v, expected
            );
        }
    }
}
