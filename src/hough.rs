use rayon::prelude::*;

use crate::grid::{GradientField, GridMatrix};

#[derive(Debug, Clone)]
pub struct HoughLine {
    pub rho: i32,
    #[allow(dead_code)]
    pub theta_idx: usize,
    pub votes: u32,
    pub theta_deg: f64,
}

#[derive(Debug, Clone)]
pub struct DefectRegion {
    pub center_row: usize,
    pub center_col: usize,
    pub radius_rows: usize,
    pub radius_cols: usize,
    pub peak_votes: u32,
    pub classification: DefectClass,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DefectClass {
    Hyperbolic,
    Elliptical,
    Linear,
    Unknown,
}

pub struct HoughDetector {
    pub rho_resolution: f64,
    pub theta_resolution_deg: f64,
    pub edge_threshold: f32,
    pub vote_threshold: u32,
    pub nms_window: usize,
}

impl HoughDetector {
    pub fn new() -> Self {
        Self {
            rho_resolution: 1.0,
            theta_resolution_deg: 1.0,
            edge_threshold: 30.0,
            vote_threshold: 50,
            nms_window: 9,
        }
    }

    pub fn detect(&mut self, grid: &GridMatrix) -> Vec<DefectRegion> {
        let grad = GradientField::compute(grid);
        let edges = grad.edge_pixels(self.edge_threshold);
        if edges.is_empty() {
            return Vec::new();
        }

        let max_rho = ((grid.rows as f64).hypot(grid.cols as f64) / self.rho_resolution).ceil() as usize;
        let n_theta = (180.0 / self.theta_resolution_deg).ceil() as usize;
        let accumulator_size = (2 * max_rho + 1) * n_theta;

        let theta_step = self.theta_resolution_deg.to_radians();
        let rho_offset = max_rho as i32;
        let edge_threshold = self.edge_threshold;
        let rho_res = self.rho_resolution;

        let num_chunks = std::cmp::min(
            rayon::current_num_threads().max(1),
            (edges.len() / 4096).max(1),
        );
        let chunk_size = (edges.len() + num_chunks - 1) / num_chunks;

        let local_accumulators: Vec<Vec<u32>> = edges
            .par_chunks(chunk_size.max(1))
            .map(|chunk| {
                let mut local = vec![0u32; accumulator_size];
                for &(r, c) in chunk {
                    let x = c as f64;
                    let y = r as f64;
                    let idx = r * grid.cols + c;
                    let dir = if idx < grad.direction.len() {
                        grad.direction[idx]
                    } else {
                        0.0
                    };
                    let angles = search_angles_static(dir, theta_step);

                    for &theta in &angles {
                        let rho = x * theta.cos() + y * theta.sin();
                        let rho_idx = (rho / rho_res).round() as i32 + rho_offset;
                        let theta_idx = (theta / theta_step).round() as usize;
                        if rho_idx >= 0
                            && (rho_idx as usize) <= 2 * max_rho
                            && theta_idx < n_theta
                        {
                            let acc_idx = rho_idx as usize * n_theta + theta_idx;
                            local[acc_idx] += 1;
                        }
                    }
                }
                local
            })
            .collect();

        let mut accumulator: Vec<u32> = vec![0u32; accumulator_size];
        for local in &local_accumulators {
            let _ = edge_threshold;
            for i in 0..accumulator_size {
                accumulator[i] = accumulator[i].saturating_add(local[i]);
            }
        }

        let lines = self.extract_peaks(&accumulator, max_rho, n_theta);
        self.cluster_defects(&lines, grid.rows, grid.cols)
    }

    fn extract_peaks(
        &self,
        accumulator: &[u32],
        max_rho: usize,
        n_theta: usize,
    ) -> Vec<HoughLine> {
        let mut peaks: Vec<HoughLine> = Vec::new();
        let half_w = self.nms_window / 2;
        let rho_len = 2 * max_rho + 1;

        for rho_idx in half_w..rho_len.saturating_sub(half_w) {
            for theta_idx in half_w..n_theta.saturating_sub(half_w) {
                let acc_idx = rho_idx * n_theta + theta_idx;
                let val = accumulator[acc_idx];
                if val < self.vote_threshold {
                    continue;
                }
                let mut is_max = true;
                let r_start = rho_idx as i32 - half_w as i32;
                let r_end = rho_idx as i32 + half_w as i32;
                let t_start = theta_idx as i32 - half_w as i32;
                let t_end = theta_idx as i32 + half_w as i32;
                'outer: for nr in r_start..=r_end {
                    for nt in t_start..=t_end {
                        if nr == rho_idx as i32 && nt == theta_idx as i32 {
                            continue;
                        }
                        if nr >= 0
                            && (nr as usize) < rho_len
                            && nt >= 0
                            && (nt as usize) < n_theta
                        {
                            if accumulator[nr as usize * n_theta + nt as usize] > val {
                                is_max = false;
                                break 'outer;
                            }
                        }
                    }
                }
                if is_max {
                    let rho = rho_idx as i32 - max_rho as i32;
                    let theta_deg = theta_idx as f64 * self.theta_resolution_deg;
                    peaks.push(HoughLine {
                        rho,
                        theta_idx,
                        votes: val,
                        theta_deg,
                    });
                }
            }
        }
        peaks.sort_by(|a, b| b.votes.cmp(&a.votes));
        peaks
    }

    fn cluster_defects(
        &self,
        lines: &[HoughLine],
        rows: usize,
        cols: usize,
    ) -> Vec<DefectRegion> {
        if lines.is_empty() {
            return Vec::new();
        }

        let mut visited: Vec<bool> = vec![false; lines.len()];
        let mut defects: Vec<DefectRegion> = Vec::new();

        for i in 0..lines.len() {
            if visited[i] {
                continue;
            }
            visited[i] = true;
            let mut cluster: Vec<usize> = Vec::with_capacity(8);
            cluster.push(i);

            let mut queue: Vec<usize> = Vec::with_capacity(8);
            queue.push(i);

            while let Some(cur) = queue.pop() {
                for j in (cur + 1)..lines.len() {
                    if visited[j] {
                        continue;
                    }
                    let theta_diff = (lines[cur].theta_deg - lines[j].theta_deg).abs();
                    let rho_diff = (lines[cur].rho - lines[j].rho).abs() as f64;
                    if theta_diff < 20.0 && rho_diff < 30.0 {
                        visited[j] = true;
                        cluster.push(j);
                        queue.push(j);
                    }
                }
            }

            cluster.sort_by(|&a, &b| lines[b].votes.cmp(&lines[a].votes));

            let peak_line = &lines[cluster[0]];
            let theta_rad = peak_line.theta_deg.to_radians();
            let cos_t = theta_rad.cos();
            let sin_t = theta_rad.sin();
            let rho_f = peak_line.rho as f64 * self.rho_resolution;

            let center_x = rho_f * cos_t;
            let center_y = rho_f * sin_t;

            let center_col = center_x.round().clamp(0.0, (cols.saturating_sub(1)) as f64) as usize;
            let center_row = center_y.round().clamp(0.0, (rows.saturating_sub(1)) as f64) as usize;

            let spread_theta: f64 = if cluster.len() > 1 {
                let thetas: Vec<f64> = cluster.iter().map(|&idx| lines[idx].theta_deg).collect();
                let min = thetas.iter().cloned().fold(f64::INFINITY, f64::min);
                let max = thetas.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                max - min
            } else {
                0.0
            };

            let spread_rho: f64 = if cluster.len() > 1 {
                let rhos: Vec<i32> = cluster.iter().map(|&idx| lines[idx].rho).collect();
                let min_r = *rhos.iter().min().unwrap_or(&0);
                let max_r = *rhos.iter().max().unwrap_or(&0);
                (max_r - min_r) as f64
            } else {
                0.0
            };

            let classification = if spread_theta < 5.0 && spread_rho < 5.0 {
                DefectClass::Elliptical
            } else if spread_theta > 10.0 {
                DefectClass::Hyperbolic
            } else if spread_theta < 5.0 && spread_rho > 5.0 {
                DefectClass::Linear
            } else {
                DefectClass::Unknown
            };

            let rr = (spread_rho * sin_t.abs()).ceil() as usize;
            let rc = (spread_rho * cos_t.abs()).ceil() as usize;
            let radius_rows = rr.max(3).min(rows / 2);
            let radius_cols = rc.max(3).min(cols / 2);

            defects.push(DefectRegion {
                center_row,
                center_col,
                radius_rows,
                radius_cols,
                peak_votes: peak_line.votes,
                classification,
            });
        }

        defects
    }
}

fn search_angles_static(edge_dir: f32, theta_step: f64) -> [f64; 5] {
    let base = edge_dir as f64;
    let offsets = [-0.15, -0.075, 0.0, 0.075, 0.15];
    let mut out = [0.0f64; 5];
    for i in 0..5 {
        let a = base + offsets[i];
        let pi = std::f64::consts::PI;
        let a = ((a % pi) + pi) % pi;
        out[i] = (a / theta_step).round() * theta_step;
    }
    out
}

impl Default for HoughDetector {
    fn default() -> Self {
        Self::new()
    }
}
