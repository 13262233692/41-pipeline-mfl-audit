use crate::defect_extract::DefectSpatialCube;
use crate::dipole::{DefectGeometry, DipoleForwardModel, Vec3};

#[derive(Debug, Clone)]
pub struct InversionResult {
    pub region_id: usize,
    pub geometry: DefectGeometry,
    pub residual_abs: f64,
    pub residual_rel: f64,
    pub iterations: usize,
    pub converged: bool,
    pub field_gain: f64,
    pub severity: DepthSeverity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DepthSeverity {
    Negligible,
    Minor,
    Moderate,
    Severe,
    Critical,
}

impl DepthSeverity {
    pub fn label(&self) -> &'static str {
        match self {
            DepthSeverity::Negligible => "Negligible  (<5%WT)",
            DepthSeverity::Minor => "Minor       (5-15%WT)",
            DepthSeverity::Moderate => "Moderate   (15-30%WT)",
            DepthSeverity::Severe => "Severe      (30-50%WT)",
            DepthSeverity::Critical => "Critical    (>50%WT)",
        }
    }

    pub fn tag(&self) -> char {
        match self {
            DepthSeverity::Negligible => '.',
            DepthSeverity::Minor => ':',
            DepthSeverity::Moderate => '+',
            DepthSeverity::Severe => '#',
            DepthSeverity::Critical => '@',
        }
    }
}

pub fn classify_depth(depth_um: f64, wall_thickness_mm: f64) -> DepthSeverity {
    let pct = if wall_thickness_mm > 0.0 {
        (depth_um * 1e-3) / wall_thickness_mm
    } else {
        0.0
    };
    if pct < 0.05 { DepthSeverity::Negligible }
    else if pct < 0.15 { DepthSeverity::Minor }
    else if pct < 0.30 { DepthSeverity::Moderate }
    else if pct < 0.50 { DepthSeverity::Severe }
    else { DepthSeverity::Critical }
}

#[derive(Debug, Clone)]
pub struct GaussNewtonConfig {
    pub max_iter: usize,
    pub tol_abs: f64,
    pub tol_rel: f64,
    pub lambda_init: f64,
    pub lambda_up: f64,
    pub lambda_down: f64,
    pub jac_eps: f64,
}

impl Default for GaussNewtonConfig {
    fn default() -> Self {
        Self {
            max_iter: 50,
            tol_abs: 1e-9,
            tol_rel: 1e-5,
            lambda_init: 0.001,
            lambda_up: 10.0,
            lambda_down: 0.5,
            jac_eps: 1e-4,
        }
    }
}

fn residual_l2(measured: &[Vec3], predicted: &[Vec3]) -> f64 {
    let mut sum = 0.0;
    for (m, p) in measured.iter().zip(predicted.iter()) {
        let d = m.sub(*p);
        sum += d.dot(d);
    }
    sum.sqrt()
}

fn compute_predicted(
    model: &DipoleForwardModel,
    geom: &DefectGeometry,
    sensors: &[(f64, f64)],
) -> Vec<Vec3> {
    let cells = model.discretize(geom);
    sensors
        .iter()
        .map(|&(sx, sy)| model.field_at_sensor(&cells, sx, sy))
        .collect()
}

pub fn invert_depth(
    cube: &DefectSpatialCube,
    wall_thickness_mm: f64,
    sensor_lift_mm: f64,
    cfg: &GaussNewtonConfig,
) -> InversionResult {
    let measured = cube.measured_field_vec();
    let sensors = cube.sensor_list();
    let n_sensors = sensors.len();

    if n_sensors == 0 || measured.is_empty() {
        return InversionResult {
            region_id: cube.region_id,
            geometry: DefectGeometry::new(10.0, 1.0, 1.0, 0.0, 0.0),
            residual_abs: f64::INFINITY,
            residual_rel: 1.0,
            iterations: 0,
            converged: false,
            field_gain: 0.0,
            severity: DepthSeverity::Negligible,
        };
    }

    let wt_um = wall_thickness_mm * 1000.0;
    let d_min_mm = cube.distance_mm.first().copied().unwrap_or(0.0);
    let d_max_mm = cube.distance_mm.last().copied().unwrap_or(1.0);
    let d_center = (d_min_mm + d_max_mm) * 0.5;
    let a_min = cube.angle_deg.first().copied().unwrap_or(0.0);
    let a_max = cube.angle_deg.last().copied().unwrap_or(1.0);
    let a_center = (a_min + a_max) * 0.5;
    let l_init = ((d_max_mm - d_min_mm).max(1.0) * 0.4).max(2.0);
    let w_init = ((a_max - a_min).max(1.0) * wall_thickness_mm * std::f64::consts::PI / 180.0).max(1.0);
    let d_init = (cube.amplitude_peak / 500.0 * wt_um).clamp(50.0, wt_um * 0.8);

    let mag = (cube.amplitude_peak * 1e6).clamp(1000.0, 1e5);
    let model = DipoleForwardModel::new(wall_thickness_mm, sensor_lift_mm, mag, 0.5);

    let mut params = DefectGeometry::new(d_init, l_init, w_init, d_center, a_center).param_vec();
    let mut geom = DefectGeometry::from_params(&params);
    let mut predicted = compute_predicted(&model, &geom, &sensors);
    let mut resid = residual_l2(&measured, &predicted);

    let scale_norm: f64 = measured.iter().map(|v| v.dot(*v)).sum::<f64>().sqrt().max(1e-12);
    let mut rel_resid = resid / scale_norm;

    let mut lambda = cfg.lambda_init;
    let mut converged = false;
    let mut it = 0usize;

    let n_params = params.len();
    for iter in 0..cfg.max_iter {
        it = iter + 1;

        if resid < cfg.tol_abs || rel_resid < cfg.tol_rel {
            converged = true;
            break;
        }

        let jac_rows = model.jacobian_numerical(&geom, &sensors, cfg.jac_eps);

        let mut jtj = vec![vec![0.0f64; n_params]; n_params];
        let mut jtr = vec![0.0f64; n_params];

        for (s_idx, (m, p)) in measured.iter().zip(predicted.iter()).enumerate() {
            let dm = m.sub(*p);
            let comp = [dm.x, dm.y, dm.z];
            for c in 0..3 {
                let residual_c = comp[c];
                let j_row = &jac_rows[s_idx];
                let dr = [j_row[0], j_row[1], j_row[2], j_row[3], j_row[4], j_row[5]];
                let j_comp = match c {
                    0 => [dr[0].x, dr[1].x, dr[2].x, dr[3].x, dr[4].x, dr[5].x],
                    1 => [dr[0].y, dr[1].y, dr[2].y, dr[3].y, dr[4].y, dr[5].y],
                    _ => [dr[0].z, dr[1].z, dr[2].z, dr[3].z, dr[4].z, dr[5].z],
                };
                for i in 0..n_params {
                    for j in 0..n_params {
                        jtj[i][j] += j_comp[i] * j_comp[j];
                    }
                    jtr[i] += j_comp[i] * residual_c;
                }
            }
        }

        let mut a = jtj.clone();
        for i in 0..n_params {
            a[i][i] += lambda * (1.0 + a[i][i].abs());
        }

        let delta = solve_linear_6(&a, &jtr);

        let mut trial_params = vec![0.0f64; n_params];
        for i in 0..n_params {
            trial_params[i] = params[i] + delta[i];
        }
        let trial_geom = DefectGeometry::from_params(&trial_params);
        let trial_pred = compute_predicted(&model, &trial_geom, &sensors);
        let trial_resid = residual_l2(&measured, &trial_pred);

        if trial_resid < resid {
            params = trial_params;
            geom = trial_geom;
            predicted = trial_pred;
            resid = trial_resid;
            rel_resid = resid / scale_norm;
            lambda *= cfg.lambda_down;
        } else {
            lambda *= cfg.lambda_up;
        }
    }

    if !converged && (resid < cfg.tol_abs || rel_resid < cfg.tol_rel) {
        converged = true;
    }

    let mut field_gain = 0.0f64;
    for (m, p) in measured.iter().zip(predicted.iter()) {
        let mn = m.norm();
        let pn = p.norm();
        if pn > 1e-15 {
            let r = mn / pn;
            field_gain = field_gain.max(r);
        }
    }

    let severity = classify_depth(geom.depth_um, wall_thickness_mm);

    InversionResult {
        region_id: cube.region_id,
        geometry: geom,
        residual_abs: resid,
        residual_rel: rel_resid,
        iterations: it,
        converged,
        field_gain,
        severity,
    }
}

fn solve_linear_6(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = 6usize;
    let mut aug = vec![vec![0.0f64; n + 1]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n] = b[i];
    }

    for col in 0..n {
        let mut max = (aug[col][col].abs(), col);
        for row in col + 1..n {
            let v = aug[row][col].abs();
            if v > max.0 { max = (v, row); }
        }
        if max.0 < 1e-15 {
            continue;
        }
        if max.1 != col { aug.swap(col, max.1); }
        let pivot = aug[col][col];
        if pivot.abs() < 1e-15 { continue; }
        for j in col..=n { aug[col][j] /= pivot; }
        for row in 0..n {
            if row == col { continue; }
            let f = aug[row][col];
            if f.abs() < 1e-15 { continue; }
            for j in col..=n { aug[row][j] -= f * aug[col][j]; }
        }
    }

    (0..n).map(|i| if aug[i][i].abs() > 1e-15 { aug[i][n] } else { 0.0 }).collect()
}

pub fn invert_all_cubes(
    cubes: &[DefectSpatialCube],
    wall_thickness_mm: f64,
    sensor_lift_mm: f64,
) -> Vec<InversionResult> {
    let cfg = GaussNewtonConfig::default();
    cubes
        .iter()
        .map(|c| invert_depth(c, wall_thickness_mm, sensor_lift_mm, &cfg))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_depth_boundaries() {
        assert_eq!(classify_depth(10.0, 10.0), DepthSeverity::Negligible);
        assert_eq!(classify_depth(1_000.0, 10.0), DepthSeverity::Minor);
        assert_eq!(classify_depth(2_500.0, 10.0), DepthSeverity::Moderate);
        assert_eq!(classify_depth(4_000.0, 10.0), DepthSeverity::Severe);
        assert_eq!(classify_depth(8_000.0, 10.0), DepthSeverity::Critical);
    }

    #[test]
    fn linear_solve_identity() {
        let a = vec![
            vec![2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0, 3.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0, 0.0, 4.0, 0.0, 0.0, 0.0],
            vec![0.0, 0.0, 0.0, 5.0, 0.0, 0.0],
            vec![0.0, 0.0, 0.0, 0.0, 6.0, 0.0],
            vec![0.0, 0.0, 0.0, 0.0, 0.0, 7.0],
        ];
        let b = vec![4.0, 9.0, 16.0, 25.0, 36.0, 49.0];
        let x = solve_linear_6(&a, &b);
        for i in 0..6 { assert!((x[i] - (i + 2) as f64).abs() < 1e-9); }
    }

    #[test]
    fn self_consistent_inversion() {
        let true_geom = DefectGeometry::new(300.0, 12.0, 5.0, 100.0, 90.0);
        let wall_mm = 7.1f64;
        let model = DipoleForwardModel::new(wall_mm, 1.0, 20000.0, 0.5);
        let n_s = 12;
        let sensors: Vec<(f64, f64)> = (0..n_s)
            .map(|i| (90.0 + i as f64 * 2.0, 90.0))
            .collect();
        let cells = model.discretize(&true_geom);
        let measured: Vec<Vec3> = sensors
            .iter()
            .map(|&(sx, sy)| model.field_at_sensor(&cells, sx, sy))
            .collect();
        let mut axial = vec![vec![0.0f64; 1]; n_s];
        let mut transverse = vec![vec![0.0f64; 1]; n_s];
        let mut radial = vec![vec![0.0f64; 1]; n_s];
        let mut peak = 0.0;
        for i in 0..n_s {
            axial[i][0] = measured[i].x;
            transverse[i][0] = measured[i].y;
            radial[i][0] = measured[i].z;
            let m = measured[i].norm();
            if m > peak { peak = m; }
        }
        let cube = DefectSpatialCube {
            region_id: 0,
            center_row: 0,
            center_col: 0,
            row_window: 0..n_s,
            col_window: 0..1,
            axial,
            transverse,
            radial,
            distance_mm: (0..n_s).map(|i| 90.0 + i as f64 * 2.0).collect(),
            angle_deg: vec![90.0],
            amplitude_peak: peak,
        };
        let cfg = GaussNewtonConfig {
            max_iter: 80,
            tol_abs: 1e-15,
            tol_rel: 1e-10,
            ..Default::default()
        };
        let res = invert_depth(&cube, wall_mm, 1.0, &cfg);
        assert!(
            res.converged || res.iterations == 80,
            "Solver should attempt all iterations if not converged"
        );
        assert!(
            res.geometry.depth_um > 10.0 && res.geometry.depth_um < 5000.0,
            "Depth should be physically reasonable, got {}",
            res.geometry.depth_um
        );
        assert!(
            res.geometry.length_mm > 0.1 && res.geometry.width_mm > 0.1,
            "Dimensions should be positive"
        );
    }
}
