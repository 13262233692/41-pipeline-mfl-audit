use crate::stream_parser::MflStream;

pub struct VelocityCorrector {
    pub nominal_velocity_mm_s: f32,
    pub max_warp: f32,
}

impl VelocityCorrector {
    pub fn new(nominal: f32) -> Self {
        Self {
            nominal_velocity_mm_s: nominal,
            max_warp: 2.0,
        }
    }

    pub fn correct_channel(
        &self,
        stream: &MflStream,
        channel: u16,
        axis: u8,
    ) -> Vec<f32> {
        let n_frames = stream.num_frames();
        if n_frames == 0 {
            return Vec::new();
        }

        let mut raw: Vec<f32> = Vec::with_capacity(n_frames);
        let mut dist: Vec<f32> = Vec::with_capacity(n_frames);
        for i in 0..n_frames {
            if let Some(frame) = stream.frame(i) {
                let axes = stream.decode_channel_axes(&frame, channel);
                let idx = axis as usize;
                raw.push(if idx < axes.len() { axes[idx] as f32 } else { 0.0 });
                dist.push(frame.hdr.distance_mm);
            }
        }

        if raw.is_empty() {
            return raw;
        }

        let mut corrected = Vec::with_capacity(raw.len());
        corrected.push(raw[0]);

        let mut prev_d = dist[0];
        for i in 1..raw.len() {
            let dd = dist[i] - prev_d;
            if dd <= 0.0 {
                corrected.push(raw[i]);
                prev_d = dist[i];
                continue;
            }
            let warp = dd * self.nominal_velocity_mm_s;
            let warp = warp.clamp(1.0 / self.max_warp, self.max_warp);
            let n_interp = (warp + 0.5) as usize;
            if n_interp <= 1 {
                corrected.push(raw[i]);
            } else {
                let step = 1.0 / n_interp as f32;
                let delta = raw[i] - raw[i - 1];
                for j in 1..n_interp {
                    let t = j as f32 * step;
                    corrected.push(raw[i - 1] + delta * t);
                }
            }
            prev_d = dist[i];
        }
        corrected
    }
}

pub struct AdaptiveMedianBgRemover {
    pub window: usize,
    pub threshold_sigma: f32,
}

impl AdaptiveMedianBgRemover {
    pub fn new(window: usize, threshold_sigma: f32) -> Self {
        Self {
            window,
            threshold_sigma,
        }
    }

    pub fn remove(&self, signal: &[f32]) -> Vec<f32> {
        if signal.is_empty() {
            return Vec::new();
        }
        let n = signal.len();
        let half = self.window / 2;
        let mut bg = vec![0.0f32; n];

        let mut buf: Vec<f32> = Vec::with_capacity(self.window);
        for i in 0..n {
            buf.clear();
            let lo = if i >= half { i - half } else { 0 };
            let hi = (i + half + 1).min(n);
            buf.extend_from_slice(&signal[lo..hi]);
            buf.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let mid = buf.len() / 2;
            bg[i] = if buf.len() % 2 == 0 {
                (buf[mid - 1] + buf[mid]) * 0.5
            } else {
                buf[mid]
            };
        }

        let mut residual: Vec<f32> = signal
            .iter()
            .zip(bg.iter())
            .map(|(s, b)| s - b)
            .collect();

        if self.threshold_sigma > 0.0 {
            let mean: f32 = residual.iter().sum::<f32>() / residual.len() as f32;
            let var: f32 = residual.iter().map(|v| (v - mean).powi(2)).sum::<f32>()
                / residual.len() as f32;
            let std = var.sqrt();
            let lo = mean - self.threshold_sigma * std;
            let hi = mean + self.threshold_sigma * std;
            for v in residual.iter_mut() {
                if *v < lo || *v > hi {
                    *v = 0.0;
                }
            }
        }

        residual
    }
}

pub fn full_preprocess(
    stream: &MflStream,
    channel: u16,
    axis: u8,
    nominal_vel: f32,
    med_window: usize,
    sigma_thresh: f32,
) -> Vec<f32> {
    let vc = VelocityCorrector::new(nominal_vel);
    let corrected = vc.correct_channel(stream, channel, axis);
    let remover = AdaptiveMedianBgRemover::new(med_window, sigma_thresh);
    remover.remove(&corrected)
}
