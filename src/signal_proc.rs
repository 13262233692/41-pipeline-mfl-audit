pub struct ChannelOwnedBuffer {
    pub samples: Vec<f32>,
    pub distances: Vec<f32>,
    pub frame_count: usize,
}

pub struct VelocityCorrector {
    pub nominal_velocity_mm_s: f32,
    pub max_warp: f32,
    pub distance_jump_threshold_mm: f32,
}

impl VelocityCorrector {
    pub fn new(nominal: f32) -> Self {
        Self {
            nominal_velocity_mm_s: nominal,
            max_warp: 2.0,
            distance_jump_threshold_mm: 1000.0,
        }
    }

    pub fn load_channel_from_owned(
        &self,
        stream: &crate::stream_parser::MflStream,
        channel: u16,
        axis: u8,
    ) -> ChannelOwnedBuffer {
        let n_frames = stream.num_frames();
        let mut samples = Vec::<f32>::with_capacity(n_frames);
        let mut distances = Vec::<f32>::with_capacity(n_frames);
        let mut last_valid_dist: Option<f32> = None;

        for i in 0..n_frames {
            if let Some(frame) = stream.frame(i) {
                let axes = stream.decode_channel_axes(&frame, channel);
                let idx = axis as usize;
                let s = if idx < axes.len() {
                    axes[idx] as f32
                } else {
                    0.0
                };
                let raw_d = frame.hdr.distance_mm;

                let d = match last_valid_dist {
                    Some(prev) => {
                        let delta = raw_d - prev;
                        if delta < 0.0 || delta > self.distance_jump_threshold_mm {
                            prev
                        } else {
                            last_valid_dist = Some(raw_d);
                            raw_d
                        }
                    }
                    None => {
                        last_valid_dist = Some(raw_d);
                        raw_d
                    }
                };

                samples.push(s);
                distances.push(d);
            }
        }

        ChannelOwnedBuffer {
            frame_count: samples.len(),
            samples,
            distances,
        }
    }

    pub fn correct_channel(
        &self,
        stream: &crate::stream_parser::MflStream,
        channel: u16,
        axis: u8,
    ) -> Vec<f32> {
        let buf = self.load_channel_from_owned(stream, channel, axis);
        self.correct_owned(&buf)
    }

    pub fn correct_owned(&self, buf: &ChannelOwnedBuffer) -> Vec<f32> {
        let raw = &buf.samples;
        let dist = &buf.distances;
        let n = raw.len();
        if n == 0 {
            return Vec::new();
        }

        let max_warp_usize = self.max_warp.ceil() as usize;
        let capacity = n * max_warp_usize.max(1);
        let mut corrected = Vec::<f32>::with_capacity(capacity);

        corrected.push(raw[0]);

        let mut prev_d = dist[0];
        let cap_limit = capacity as isize;

        for i in 1..n {
            let dd = dist[i] - prev_d;
            if dd <= f32::EPSILON {
                if corrected.len() as isize >= cap_limit {
                    break;
                }
                corrected.push(raw[i]);
                prev_d = dist[i];
                continue;
            }

            let warp = dd * self.nominal_velocity_mm_s;
            let warp = warp.clamp(1.0 / self.max_warp, self.max_warp);
            let n_interp = ((warp + 0.5) as usize).max(1).min(max_warp_usize + 1);

            let remaining = cap_limit - corrected.len() as isize;
            let n_interp = n_interp.min(remaining as usize);

            if n_interp <= 1 || remaining <= 0 {
                if remaining > 0 {
                    corrected.push(raw[i]);
                }
            } else {
                let step = 1.0 / n_interp as f32;
                let delta = raw[i] - raw[i - 1];
                for j in 1..n_interp {
                    let t = j as f32 * step;
                    corrected.push(raw[i - 1] + delta * t);
                    if corrected.len() as isize >= cap_limit {
                        break;
                    }
                }
                if (corrected.len() as isize) < cap_limit {
                    corrected.push(raw[i]);
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
        let n = signal.len();
        if n == 0 {
            return Vec::new();
        }
        let half = self.window / 2;
        let mut bg = vec![0.0f32; n];

        let mut scratch: Vec<f32> = Vec::with_capacity(self.window + 1);

        for i in 0..n {
            scratch.clear();
            let lo = if i >= half { i - half } else { 0 };
            let hi = (i + half + 1).min(n);
            scratch.extend_from_slice(&signal[lo..hi]);

            scratch.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

            let mid = scratch.len() / 2;
            bg[i] = if scratch.len() % 2 == 0 && mid > 0 {
                (scratch[mid - 1] + scratch[mid]) * 0.5
            } else {
                scratch[mid]
            };
        }

        let mut residual = vec![0.0f32; n];
        for i in 0..n {
            residual[i] = signal[i] - bg[i];
        }

        if self.threshold_sigma > 0.0 && n > 1 {
            let mut sum: f64 = 0.0;
            for &v in residual.iter() {
                sum += v as f64;
            }
            let mean = (sum / n as f64) as f32;

            let mut var_sum: f64 = 0.0;
            for &v in residual.iter() {
                let d = (v - mean) as f64;
                var_sum += d * d;
            }
            let std = (var_sum / n as f64).sqrt() as f32;

            if std > f32::EPSILON {
                let lo = mean - self.threshold_sigma * std;
                let hi = mean + self.threshold_sigma * std;
                for v in residual.iter_mut() {
                    if *v < lo || *v > hi {
                        *v = 0.0;
                    }
                }
            }
        }

        residual
    }
}

pub fn full_preprocess(
    stream: &crate::stream_parser::MflStream,
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
