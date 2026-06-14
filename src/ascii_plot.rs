use crate::depth_inversion::{DepthSeverity, InversionResult};
use crate::hough::DefectRegion;

pub struct CorrosionMapConfig {
    pub pipe_width_chars: usize,
    pub min_window_mm: f64,
    pub max_window_mm: f64,
    pub step_distance_mm: f64,
    pub wall_thickness_mm: f64,
}

impl Default for CorrosionMapConfig {
    fn default() -> Self {
        Self {
            pipe_width_chars: 60,
            min_window_mm: 0.0,
            max_window_mm: f64::INFINITY,
            step_distance_mm: 1.0,
            wall_thickness_mm: 7.1,
        }
    }
}

fn severity_block_char(s: DepthSeverity) -> char {
    s.tag()
}

fn severity_block(s: DepthSeverity) -> &'static str {
    match s {
        DepthSeverity::Negligible => "  ",
        DepthSeverity::Minor => "· ",
        DepthSeverity::Moderate => "+ ",
        DepthSeverity::Severe => "##",
        DepthSeverity::Critical => "@@",
    }
}

pub fn print_ascii_pipe_map(
    results: &[InversionResult],
    regions: &[DefectRegion],
    cfg: &CorrosionMapConfig,
    total_distance_mm: f64,
) {
    let total = total_distance_mm.max(1.0).min(cfg.max_window_mm);
    let start = cfg.min_window_mm.max(0.0);
    let range = (total - start).max(1.0);

    let width_c = cfg.pipe_width_chars.max(20);
    let len_c = ((range / cfg.step_distance_mm.max(1.0)).ceil() as usize).min(200).max(10);

    let mut grid: Vec<Vec<DepthSeverity>> = vec![vec![DepthSeverity::Negligible; len_c]; width_c];

    let mut placed: Vec<bool> = vec![false; results.len()];

    for (i, res) in results.iter().enumerate() {
        let x = res.geometry.pos_x_mm.clamp(start, total);
        let col_abs = ((x - start) / range * len_c as f64).floor() as usize;
        let col = col_abs.min(len_c - 1);

        let y_angle = res.geometry.pos_y_mm.max(0.0) % 360.0;
        let row_abs = (y_angle / 360.0 * width_c as f64).floor() as usize;
        let row = row_abs.min(width_c - 1);

        let h_sem = (res.geometry.length_mm * 0.5 / range * len_c as f64)
            .ceil()
            .max(1.0) as usize;
        let w_sem = (res.geometry.width_mm / (std::f64::consts::PI * cfg.wall_thickness_mm.max(0.1))
            * width_c as f64 * 0.5)
            .ceil()
            .max(1.0) as usize;

        for dr in 0..=w_sem {
            for dc in 0..=h_sem {
                let local_norm = (dr as f64 / w_sem.max(1) as f64).powi(2)
                    + (dc as f64 / h_sem.max(1) as f64).powi(2);
                if local_norm > 1.0 { continue; }
                let local_sev = if local_norm < 0.25 {
                    res.severity
                } else if local_norm < 0.55 {
                    match res.severity {
                        DepthSeverity::Critical => DepthSeverity::Severe,
                        DepthSeverity::Severe => DepthSeverity::Moderate,
                        DepthSeverity::Moderate => DepthSeverity::Minor,
                        other => other,
                    }
                } else {
                    match res.severity {
                        DepthSeverity::Critical => DepthSeverity::Moderate,
                        DepthSeverity::Severe => DepthSeverity::Minor,
                        _ => DepthSeverity::Negligible,
                    }
                };
                let local_sev_val = local_sev;
                let cand_rank = match local_sev {
                    DepthSeverity::Negligible => 0,
                    DepthSeverity::Minor => 1,
                    DepthSeverity::Moderate => 2,
                    DepthSeverity::Severe => 3,
                    DepthSeverity::Critical => 4,
                };
                let cell_set = |grid: &mut Vec<Vec<DepthSeverity>>, r: usize, c: usize| {
                    if r < width_c && c < len_c {
                        let cur_rank = match grid[r][c] {
                            DepthSeverity::Negligible => 0,
                            DepthSeverity::Minor => 1,
                            DepthSeverity::Moderate => 2,
                            DepthSeverity::Severe => 3,
                            DepthSeverity::Critical => 4,
                        };
                        if cand_rank > cur_rank || matches!(grid[r][c], DepthSeverity::Negligible) {
                            grid[r][c] = local_sev_val;
                        }
                    }
                };
                cell_set(&mut grid, row.saturating_add(dr), col.saturating_add(dc));
                cell_set(&mut grid, row.saturating_add(dr), col.saturating_sub(dc));
                if dr > 0 { cell_set(&mut grid, row.saturating_sub(dr), col.saturating_add(dc)); }
                if dc > 0 { cell_set(&mut grid, row.saturating_add(dr), col.saturating_sub(dc)); }
                if dr > 0 && dc > 0 { cell_set(&mut grid, row.saturating_sub(dr), col.saturating_sub(dc)); }
            }
        }
        placed[i] = true;
    }

    let banner = "══════════════════════════════════════════════════════════════════════════════════";
    println!("\n{}", banner);
    println!(
        "║   PIPE INNER-WALL CORROSION DEGRADATION TOPOLOGY MAP   [{}km / WT={}mm]  ║",
        format_num_m(total),
        cfg.wall_thickness_mm
    );
    println!("{}", banner);

    println!();
    println!("   LEGEND:  [.] Negligible <5%WT   [:] Minor 5-15%   [+] Moderate 15-30%   [#] Severe 30-50%   [@] Critical >50%");
    println!();

    let tick_step_c = len_c / 10.max(1);
    let mut top_tick = String::from("   km-> ");
    for tc in 0..=10 {
        let pos = (tc * tick_step_c).min(len_c - 1);
        let d = start + (pos as f64 / len_c as f64) * range;
        top_tick.push_str(&format!("{:>7}", format_num_m(d)));
    }
    println!("{}", top_tick);

    let horiz = "─".repeat(len_c * 2 + 2);
    println!("    ╔{}╗", horiz);

    for (r, row) in grid.iter().enumerate() {
        let angle_lab = (r as f64 / width_c as f64 * 360.0) as isize;
        let angle_annot = if r % (width_c / 8).max(1) == 0 {
            format!(" {:>3}° ", angle_lab)
        } else {
            String::from("      ")
        };
        print!("{}║", angle_annot);
        for cell in row.iter() {
            print!("{}", severity_block(*cell));
        }
        println!("║");
    }
    println!("    ╚{}╝", horiz);

    let ticks_per_row = width_c / 8;
    let tick_chars: Vec<usize> = (0..=8).map(|i| (i * ticks_per_row).min(width_c - 1)).collect();
    let mut bot_tick = String::from("  12h  ");
    for &tc in &tick_chars {
        bot_tick.push_str(&format!("{:>7}", (tc as f64 / width_c as f64 * 360.0).round() as i32));
    }
    println!("{}", bot_tick);

    println!("\n{}", banner);
    println!("║                           TOP-10 INDIVIDUAL DEFECT SPECIMEN                         ║");
    println!("{}", banner);
    println!(
        " {:>3} │ {:>10} {:>8} {:>8} {:>9} {:>8} │ {:>10} {:>8} {:>8} {:>7} │ {}",
        "#", "Dist(mm)", "Len(mm)", "Wid(mm)", "Depth(μm)", "%WT", "Vol(mm³)", "Iter", "|Δ|res", "Conv.", "Severity"
    );
    println!("─────┼───────────────────────────────────────────────┼──────────────────────────────────┼────────────────────");

    let mut idx: Vec<usize> = (0..results.len()).collect();
    idx.sort_by(|&a, &b| {
        let ra = results[a].geometry.depth_um;
        let rb = results[b].geometry.depth_um;
        rb.partial_cmp(&ra).unwrap_or(std::cmp::Ordering::Equal)
    });

    for (rank, &i) in idx.iter().take(10).enumerate() {
        let r = &results[i];
        let g = &r.geometry;
        let pct = if cfg.wall_thickness_mm > 0.0 {
            g.depth_um * 1e-3 / cfg.wall_thickness_mm * 100.0
        } else {
            0.0
        };
        println!(
            " {:>3} │ {:>10.1} {:>8.2} {:>8.2} {:>9.1} {:>7.1}% │ {:>10.2} {:>8} {:>8.2e} {:>7} │ {}  {}",
            rank + 1,
            g.pos_x_mm,
            g.length_mm,
            g.width_mm,
            g.depth_um,
            pct,
            g.volume_mm3(),
            r.iterations,
            r.residual_rel,
            if r.converged { "OK" } else { "NC" },
            severity_block_char(r.severity),
            r.severity.label()
        );
    }

    let total_sev_cnt: [usize; 5] = results.iter().fold([0usize; 5], |mut acc, r| {
        match r.severity {
            DepthSeverity::Negligible => acc[0] += 1,
            DepthSeverity::Minor => acc[1] += 1,
            DepthSeverity::Moderate => acc[2] += 1,
            DepthSeverity::Severe => acc[3] += 1,
            DepthSeverity::Critical => acc[4] += 1,
        }
        acc
    });

    let total_vol: f64 = results.iter().map(|r| r.geometry.volume_mm3()).sum();
    println!("{}", banner);
    println!(
        "║ SUMMARY: TOTAL={:>4}  NEGL={:<5} MINR={:<5} MODR={:<5} SEVR={:<5} CRIT={:<5}  ΣVOL={:>9.2} mm³  ║",
        results.len(),
        total_sev_cnt[0], total_sev_cnt[1], total_sev_cnt[2], total_sev_cnt[3], total_sev_cnt[4],
        total_vol
    );
    println!("{}", banner);

    let _ = regions;
}

fn format_num_m(mm: f64) -> String {
    if mm >= 1_000_000.0 {
        format!("{:.2}", mm / 1_000_000.0)
    } else if mm >= 1000.0 {
        format!("{:.2}m", mm / 1000.0)
    } else {
        format!("{:.0}mm", mm)
    }
}

pub fn print_defect_evolution_curve(results: &[InversionResult], total_distance_mm: f64, n_bins: usize) {
    if results.is_empty() { return; }
    let nb = n_bins.max(10);
    let total = total_distance_mm.max(1.0);
    let mut bins = vec![0.0f64; nb];
    let mut bins_wt = vec![0.0f64; nb];

    for r in results {
        let x = r.geometry.pos_x_mm.clamp(0.0, total);
        let bi = ((x / total) * (nb - 1) as f64).round() as usize;
        bins[bi] += r.geometry.volume_mm3();
        bins_wt[bi] = bins_wt[bi].max(r.geometry.depth_um);
    }
    let max_vol = bins.iter().cloned().fold(0.0f64, f64::max).max(1e-6);
    let max_d = bins_wt.iter().cloned().fold(0.0f64, f64::max).max(1.0);

    let rows = 12usize;
    println!();
    println!("┌───────────────────────────── CUMULATIVE CORROSION EVOLUTION (km → Σmm³) ─────────────────────────────┐");
    println!("│ Depth │ Σ Defect Volume per distance bin                                                             │");
    println!("├───────┼──────────────────────────────────────────────────────────────────────────────────────────────┤");
    for r in 0..rows {
        let threshold_d = max_d * (rows - r) as f64 / rows as f64;
        let line_thr = (rows - r - 1) as f64 / rows as f64;
        print!("│ {:>5.0} │ ", threshold_d);
        for bi in 0..nb {
            let rel_v = bins[bi] / max_vol;
            let rel_d = bins_wt[bi] / max_d;
            let c = if rel_v >= line_thr && rel_d >= line_thr {
                '█'
            } else if rel_v >= line_thr {
                '▓'
            } else if rel_d >= line_thr {
                '▒'
            } else if (rel_v + rel_d) * 0.5 >= line_thr {
                '░'
            } else {
                ' '
            };
            print!("{}", c);
        }
        println!(" │");
    }
    println!("├───────┼──────────────────────────────────────────────────────────────────────────────────────────────┤");
    print!("│   0km │ ");
    for i in 0..nb {
        if i % (nb / 10).max(1) == 0 {
            print!("+");
        } else {
            print!("-");
        }
    }
    println!(" │");
    let end_km = total / 1_000_000.0;
    println!("│       {:<10.0}m  ← Distance Axis →  {:>10.1}km                                                        │", 0.0, end_km);
    println!("└──────────────────────────────────────────────────────────────────────────────────────────────────────┘");
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dipole::DefectGeometry;

    fn dummy_results(n: usize) -> (Vec<InversionResult>, Vec<DefectRegion>) {
        let mut r = Vec::with_capacity(n);
        let mut reg = Vec::with_capacity(n);
        for i in 0..n {
            let depth = 50.0 + i as f64 * 150.0;
            let g = DefectGeometry::new(depth, 5.0 + (i % 5) as f64, 2.0 + (i % 3) as f64, 10000.0 * i as f64, (i * 37) as f64 % 360.0);
            let sev = classify_depth_stub(depth, 7.1);
            r.push(InversionResult {
                region_id: i,
                geometry: g,
                residual_abs: 1e-10,
                residual_rel: 1e-6,
                iterations: 12,
                converged: true,
                field_gain: 1.0,
                severity: sev,
            });
            reg.push(DefectRegion {
                center_row: i * 20,
                center_col: i * 3,
                radius_rows: 10,
                radius_cols: 5,
                peak_votes: 100,
                classification: crate::hough::DefectClass::Elliptical,
            });
        }
        (r, reg)
    }
    fn classify_depth_stub(d: f64, w: f64) -> DepthSeverity {
        let pct = d * 1e-3 / w;
        if pct < 0.05 { DepthSeverity::Negligible }
        else if pct < 0.15 { DepthSeverity::Minor }
        else if pct < 0.30 { DepthSeverity::Moderate }
        else if pct < 0.50 { DepthSeverity::Severe }
        else { DepthSeverity::Critical }
    }

    #[test]
    fn map_prints_without_panic() {
        let (r, reg) = dummy_results(15);
        let cfg = CorrosionMapConfig::default();
        print_ascii_pipe_map(&r, &reg, &cfg, 150_000.0);
        print_defect_evolution_curve(&r, 150_000.0, 60);
    }
}
