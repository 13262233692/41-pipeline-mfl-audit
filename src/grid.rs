use std::ptr;

use rayon::prelude::*;

use crate::signal_proc::full_preprocess;
use crate::stream_parser::MflStream;

pub struct GridMatrix {
    pub rows: usize,
    pub cols: usize,
    pub columns: Vec<Box<[f32]>>,
}

impl GridMatrix {
    pub fn new(rows: usize, cols: usize) -> Self {
        let mut columns = Vec::with_capacity(cols);
        for _ in 0..cols {
            let col: Box<[f32]> = vec![0.0f32; rows].into_boxed_slice();
            columns.push(col);
        }
        Self { rows, cols, columns }
    }

    pub fn install_column(&mut self, col_idx: usize, data: Box<[f32]>) {
        if col_idx < self.cols {
            self.columns[col_idx] = data;
        }
    }

    #[inline]
    pub fn get(&self, r: usize, c: usize) -> f32 {
        if r < self.rows && c < self.cols {
            unsafe { *self.columns[c].as_ptr().add(r) }
        } else {
            0.0
        }
    }

    pub fn row_major_copy(&self) -> Vec<f32> {
        let total = self.rows * self.cols;
        let mut out = vec![0.0f32; total];
        for r in 0..self.rows {
            let row_base = r * self.cols;
            for (c, col) in self.columns.iter().enumerate() {
                if r < col.len() {
                    unsafe {
                        let dst = out.as_mut_ptr().add(row_base + c);
                        let src = col.as_ptr().add(r);
                        ptr::copy_nonoverlapping(src, dst, 1);
                    }
                }
            }
        }
        out
    }

    pub fn normalize_to_u8(&self) -> Vec<u8> {
        let flat = self.row_major_copy();
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for &v in flat.iter() {
            if v < min { min = v; }
            if v > max { max = v; }
        }
        let range = max - min;
        if range.abs() < f32::EPSILON {
            return vec![0u8; flat.len()];
        }
        flat.iter()
            .map(|&v| (((v - min) / range) * 255.0).clamp(0.0, 255.0) as u8)
            .collect()
    }
}

fn process_channel(
    stream: &MflStream,
    channel: u16,
    axis: u8,
    nominal_vel: f32,
    med_window: usize,
    sigma_thresh: f32,
    expected_rows: usize,
) -> Box<[f32]> {
    let processed = full_preprocess(stream, channel, axis, nominal_vel, med_window, sigma_thresh);
    let n = processed.len();
    let mut col: Box<[f32]> = vec![0.0f32; expected_rows].into_boxed_slice();
    let copy_len = n.min(expected_rows);
    if copy_len > 0 {
        unsafe {
            let src = processed.as_ptr();
            let dst = col.as_mut_ptr();
            ptr::copy_nonoverlapping(src, dst, copy_len);
        }
    }
    col
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

    let channel_range: Vec<u16> = (0..n_channels as u16).collect();

    let processed_columns: Vec<(usize, Box<[f32]>)> = channel_range
        .par_iter()
        .map(|&ch| {
            let col_box = process_channel(
                stream,
                ch,
                axis,
                nominal_vel,
                med_window,
                sigma_thresh,
                n_frames,
            );
            (ch as usize, col_box)
        })
        .collect();

    let mut grid = GridMatrix::new(n_frames, n_channels);
    for (col_idx, col_box) in processed_columns {
        grid.install_column(col_idx, col_box);
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
