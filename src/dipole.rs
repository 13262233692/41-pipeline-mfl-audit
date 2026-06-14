pub const MU0: f64 = 4.0 * std::f64::consts::PI * 1e-7;
pub const MR_STEEL: f64 = 200.0;

#[derive(Debug, Clone, Copy)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self { Self { x, y, z } }
    pub fn dot(self, other: Vec3) -> f64 { self.x * other.x + self.y * other.y + self.z * other.z }
    pub fn cross(self, other: Vec3) -> Vec3 {
        Vec3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
    pub fn norm(self) -> f64 { self.dot(self).sqrt() }
    pub fn sub(self, other: Vec3) -> Vec3 { Vec3::new(self.x - other.x, self.y - other.y, self.z - other.z) }
    pub fn add(self, other: Vec3) -> Vec3 { Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z) }
    pub fn scale(self, k: f64) -> Vec3 { Vec3::new(self.x * k, self.y * k, self.z * k) }
}

#[derive(Debug, Clone)]
pub struct DipoleCell {
    pub pos: Vec3,
    pub moment: Vec3,
    pub area: f64,
}

#[derive(Debug, Clone)]
pub struct DefectGeometry {
    pub depth_um: f64,
    pub length_mm: f64,
    pub width_mm: f64,
    pub pos_x_mm: f64,
    pub pos_y_mm: f64,
    pub tilt_rad: f64,
}

impl DefectGeometry {
    pub fn new(depth_um: f64, length_mm: f64, width_mm: f64, px: f64, py: f64) -> Self {
        Self {
            depth_um,
            length_mm,
            width_mm,
            pos_x_mm: px,
            pos_y_mm: py,
            tilt_rad: 0.0,
        }
    }

    pub fn param_vec(&self) -> Vec<f64> {
        vec![
            self.depth_um,
            self.length_mm,
            self.width_mm,
            self.pos_x_mm,
            self.pos_y_mm,
            self.tilt_rad,
        ]
    }

    pub fn from_params(params: &[f64]) -> Self {
        let get = |i: usize| if i < params.len() { params[i] } else { 0.0 };
        Self {
            depth_um: get(0).max(10.0),
            length_mm: get(1).max(0.5),
            width_mm: get(2).max(0.5),
            pos_x_mm: get(3),
            pos_y_mm: get(4),
            tilt_rad: get(5).clamp(-1.0, 1.0),
        }
    }

    pub fn volume_mm3(&self) -> f64 {
        let d_mm = self.depth_um * 1e-3;
        std::f64::consts::PI * (self.length_mm * 0.5) * (self.width_mm * 0.5) * d_mm
    }
}

pub struct DipoleForwardModel {
    pub wall_thickness_mm: f64,
    pub sensor_lift_mm: f64,
    pub background_mag_a_m: f64,
    pub cell_size_mm: f64,
}

impl DipoleForwardModel {
    pub fn new(wall_mm: f64, lift_mm: f64, mag_a_m: f64, cell_mm: f64) -> Self {
        Self {
            wall_thickness_mm: wall_mm,
            sensor_lift_mm: lift_mm,
            background_mag_a_m: mag_a_m,
            cell_size_mm: cell_mm.max(0.1),
        }
    }

    pub fn discretize(&self, geom: &DefectGeometry) -> Vec<DipoleCell> {
        let mut cells = Vec::new();
        let nx = (geom.length_mm / self.cell_size_mm).ceil() as usize;
        let ny = (geom.width_mm / self.cell_size_mm).ceil() as usize;

        let dx = if nx > 1 { geom.length_mm / (nx - 1) as f64 } else { 0.0 };
        let dy = if ny > 1 { geom.width_mm / (ny - 1) as f64 } else { 0.0 };

        let a = geom.length_mm * 0.5;
        let b = geom.width_mm * 0.5;
        let d_mm = geom.depth_um * 1e-3;
        let a_sq = if a > 0.0 { a * a } else { 1e-6 };
        let b_sq = if b > 0.0 { b * b } else { 1e-6 };

        let cos_t = geom.tilt_rad.cos();
        let sin_t = geom.tilt_rad.sin();

        for i in 0..nx {
            for j in 0..ny {
                let lx = if nx > 1 { -a + (i as f64) * dx } else { 0.0 };
                let ly = if ny > 1 { -b + (j as f64) * dy } else { 0.0 };

                let local_dx2 = (lx * lx) / a_sq + (ly * ly) / b_sq;
                if local_dx2 > 1.0 {
                    continue;
                }
                let local_depth = d_mm * (1.0 - local_dx2).sqrt().max(0.0);

                let rx = geom.pos_x_mm + lx * cos_t - ly * sin_t;
                let ry = geom.pos_y_mm + lx * sin_t + ly * cos_t;
                let rz = self.wall_thickness_mm - local_depth;

                let surf_normal_z = if d_mm > 1e-6 {
                    let nx_comp = -2.0 * lx / a_sq;
                    let ny_comp = -2.0 * ly / b_sq;
                    let nz_comp = 2.0 * local_depth / (d_mm * d_mm).max(1e-12);
                    let n_norm = (nx_comp * nx_comp + ny_comp * ny_comp + nz_comp * nz_comp).sqrt();
                    nz_comp / n_norm.max(1e-9)
                } else {
                    1.0
                };

                let cell_area = self.cell_size_mm * self.cell_size_mm;
                let m_z = self.background_mag_a_m * (1.0 - surf_normal_z.abs()) * local_depth.max(0.0)
                    / self.wall_thickness_mm.max(0.1);

                cells.push(DipoleCell {
                    pos: Vec3::new(rx * 1e-3, ry * 1e-3, rz * 1e-3),
                    moment: Vec3::new(0.0, 0.0, m_z.max(1e-9) * cell_area * 1e-6),
                    area: cell_area * 1e-6,
                });
            }
        }
        cells
    }

    pub fn field_at_sensor(&self, cells: &[DipoleCell], sensor_x_mm: f64, sensor_y_mm: f64) -> Vec3 {
        let r_sensor = Vec3::new(sensor_x_mm * 1e-3, sensor_y_mm * 1e-3, (self.wall_thickness_mm + self.sensor_lift_mm) * 1e-3);

        let mut field = Vec3::new(0.0, 0.0, 0.0);
        for cell in cells {
            let r = r_sensor.sub(cell.pos);
            let r_norm = r.norm();
            if r_norm < 1e-9 {
                continue;
            }
            let r5 = r_norm.powi(5);
            let m_dot_r = cell.moment.dot(r);

            let factor1 = MU0 / (4.0 * std::f64::consts::PI * r5);
            let factor2 = 3.0 * m_dot_r / r_norm.powi(2);

            let bx = factor1 * (factor2 * r.x - cell.moment.x);
            let by = factor1 * (factor2 * r.y - cell.moment.y);
            let bz = factor1 * (factor2 * r.z - cell.moment.z);

            field.x += bx;
            field.y += by;
            field.z += bz;
        }
        field
    }

    pub fn sensor_grid(&self, geom: &DefectGeometry, rows: usize, cols: usize) -> Vec<Vec<Vec3>> {
        let cells = self.discretize(geom);
        let mut grid = Vec::with_capacity(rows);

        let x_min = (geom.pos_x_mm - geom.length_mm - 20.0).max(0.0);
        let x_max = geom.pos_x_mm + geom.length_mm + 20.0;
        let y_step = 360.0 / cols.max(1) as f64;

        for r in 0..rows {
            let mut row = Vec::with_capacity(cols);
            let xf = if rows > 1 { r as f64 / (rows - 1) as f64 } else { 0.5 };
            let sx = x_min + xf * (x_max - x_min);
            for c in 0..cols {
                let sy = (c as f64) * y_step;
                row.push(self.field_at_sensor(&cells, sx, sy));
            }
            grid.push(row);
        }
        grid
    }

    pub fn flux_integral(&self, geom: &DefectGeometry) -> (f64, f64, f64) {
        let cells = self.discretize(geom);
        let samples = 50usize;
        let r = (geom.length_mm + 10.0) as usize;
        let x0 = geom.pos_x_mm - r as f64;
        let mut ix = 0.0f64;
        let mut iy = 0.0f64;
        let mut iz = 0.0f64;

        for s in 0..samples {
            let xf = s as f64 / samples as f64;
            let sx = x0 + xf * (2.0 * r as f64);
            let f = self.field_at_sensor(&cells, sx, geom.pos_y_mm);
            let step = 2.0 * r as f64 / samples as f64;
            ix += f.x * step;
            iy += f.y * step;
            iz += f.z * step;
        }
        (ix, iy, iz)
    }

    pub fn jacobian_numerical(
        &self,
        geom: &DefectGeometry,
        sensors: &[(f64, f64)],
        eps_frac: f64,
    ) -> Vec<Vec<Vec3>> {
        let params = geom.param_vec();
        let n_params = params.len();
        let n_sensors = sensors.len();
        let mut jac = vec![vec![Vec3::new(0.0, 0.0, 0.0); n_params]; n_sensors];

        for p in 0..n_params {
            let mut p_plus = params.clone();
            let mut p_minus = params.clone();
            let base = params[p].abs();
            let eps = (base * eps_frac).max(1e-6);
            p_plus[p] += eps;
            p_minus[p] -= eps;

            let g_plus = DefectGeometry::from_params(&p_plus);
            let g_minus = DefectGeometry::from_params(&p_minus);
            let cells_p = self.discretize(&g_plus);
            let cells_m = self.discretize(&g_minus);

            for (si, &(sx, sy)) in sensors.iter().enumerate() {
                let bp = self.field_at_sensor(&cells_p, sx, sy);
                let bm = self.field_at_sensor(&cells_m, sx, sy);
                let denom = 2.0 * eps;
                jac[si][p] = Vec3::new(
                    (bp.x - bm.x) / denom,
                    (bp.y - bm.y) / denom,
                    (bp.z - bm.z) / denom,
                );
            }
        }
        jac
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec3_basic() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert!((a.dot(b) - 32.0).abs() < 1e-9);
        let c = a.cross(b);
        assert!((c.x + 3.0).abs() < 1e-9);
        assert!((c.y - 6.0).abs() < 1e-9);
        assert!((c.z + 3.0).abs() < 1e-9);
    }

    #[test]
    fn defect_param_roundtrip() {
        let g = DefectGeometry::new(500.0, 20.0, 8.0, 100.0, 45.0);
        let p = g.param_vec();
        let g2 = DefectGeometry::from_params(&p);
        assert!((g.depth_um - g2.depth_um).abs() < 1e-6);
        assert!((g.length_mm - g2.length_mm).abs() < 1e-6);
    }

    #[test]
    fn forward_model_nonzero_field() {
        let model = DipoleForwardModel::new(7.1, 1.0, 10000.0, 1.0);
        let geom = DefectGeometry::new(300.0, 15.0, 6.0, 50.0, 180.0);
        let cells = model.discretize(&geom);
        assert!(!cells.is_empty(), "Must generate at least one dipole cell");

        let field = model.field_at_sensor(&cells, 50.0, 180.0);
        let norm = field.norm();
        assert!(norm > 1e-12, "Field should be nonzero for a real defect");
    }

    #[test]
    fn jacobian_shape_correct() {
        let model = DipoleForwardModel::new(7.1, 1.0, 10000.0, 2.0);
        let geom = DefectGeometry::new(200.0, 10.0, 4.0, 50.0, 90.0);
        let sensors = vec![(50.0, 90.0), (55.0, 90.0), (60.0, 90.0)];
        let jac = model.jacobian_numerical(&geom, &sensors, 1e-4);
        assert_eq!(jac.len(), 3);
        assert_eq!(jac[0].len(), 6);
    }
}
