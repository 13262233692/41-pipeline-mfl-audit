use crate::stream_parser::MflStream;
use crate::signal_proc::full_preprocess;

pub struct GridMatrix {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<f32>,
}

impl GridMatrix {
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    pub fn get(&self, r: usize, c: usize) -> f32 {
        self.data[r * self.cols + c]
    }

    pub fn set(&mut self, r: usize, c: usize, v: f32) {
        self.data[r * self.cols + c] = v;
    }

    pub fn normalize_to_u8(&self) -> Vec<u8> {
        let min = self.data.iter().cloned().fold(f32::INFINITY, f32::min);
        let max = self.data.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let range = max - min;
        if range.abs() < f32::EPSILON {
            return vec![0u8; self.data.len()];
        }
        self.data
            .iter()
            .map(|&v| (((v - min) / range) * 255.0).clamp(0.0, 255.0) as u8)
            .collect()
    }
}

pub fn build_grid(
    stream: &MflStream,
    axis: u8,
    nominal_vel: f32,
    med_window: usize,
    sigma_thresh: f32,
) -> GridMatrix {
    let n_channels = stream.header().num_channels as usize;
    let n_frames = stream.num_frames();
    let mut grid = GridMatrix::new(n_frames, n_channels);

    for ch in 0..n_channels {
        let ch_u16 = ch as u16;
        let processed = full_preprocess(stream, ch_u16, axis, nominal_vel, med_window, sigma_thresh);
        for (row, &val) in processed.iter().enumerate() {
            if row < n_frames {
                grid.set(row, ch, val);
            }
        }
    }
    grid
}

pub struct GradientField {
    pub rows: usize,
    pub cols: usize,
    #[allow(dead_code)]
    pub gx: Vec<f32>,
    #[allow(dead_code)]
    pub gy: Vec<f32>,
    pub magnitude: Vec<f32>,
    pub direction: Vec<f32>,
}

impl GradientField {
    pub fn compute(grid: &GridMatrix) -> Self {
        let rows = grid.rows;
        let cols = grid.cols;
        let total = rows * cols;
        let mut gx = vec![0.0f32; total];
        let mut gy = vec![0.0f32; total];
        let mut magnitude = vec![0.0f32; total];
        let mut direction = vec![0.0f32; total];

        for r in 1..rows.saturating_sub(1) {
            for c in 1..cols.saturating_sub(1) {
                let idx = r * cols + c;
                let dx = grid.get(r, c + 1) - grid.get(r, c - 1);
                let dy = grid.get(r + 1, c) - grid.get(r - 1, c);
                gx[idx] = dx * 0.5;
                gy[idx] = dy * 0.5;
                magnitude[idx] = (dx * dx + dy * dy).sqrt();
                direction[idx] = dy.atan2(dx);
            }
        }
        Self {
            rows,
            cols,
            gx,
            gy,
            magnitude,
            direction,
        }
    }

    pub fn edge_pixels(&self, threshold: f32) -> Vec<(usize, usize)> {
        let mut edges = Vec::new();
        for r in 0..self.rows {
            for c in 0..self.cols {
                let idx = r * self.cols + c;
                if self.magnitude[idx] >= threshold {
                    edges.push((r, c));
                }
            }
        }
        edges
    }
}
