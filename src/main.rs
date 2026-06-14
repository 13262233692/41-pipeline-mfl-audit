use std::path::PathBuf;

use clap::Parser;

use mfl_audit::stream_parser::MflStream;
use mfl_audit::grid::build_grid;
use mfl_audit::hough::{HoughDetector, DefectClass};
use mfl_audit::defect_extract::extract_defect_cubes;
use mfl_audit::depth_inversion::invert_all_cubes;
use mfl_audit::ascii_plot::{print_ascii_pipe_map, print_defect_evolution_curve, CorrosionMapConfig};

#[derive(Parser, Debug)]
#[command(name = "mfl-audit", about = "Pipeline MFL raw-data discrete audit tool")]
struct Cli {
    #[arg(help = "Path to the .mfl raw data file")]
    input: PathBuf,

    #[arg(long, default_value = "1", help = "Axis index: 1=Axial 2=Transverse 3=Radial")]
    axis: u8,

    #[arg(long, default_value_t = 500.0, help = "Nominal PIG velocity in mm/s")]
    nominal_velocity: f32,

    #[arg(long, default_value_t = 21, help = "Median background subtraction window size")]
    median_window: usize,

    #[arg(long, default_value_t = 3.0, help = "Sigma threshold for noise gating")]
    sigma_threshold: f32,

    #[arg(long, default_value_t = 30.0, help = "Gradient magnitude threshold for edge detection")]
    edge_threshold: f32,

    #[arg(long, default_value_t = 50, help = "Minimum Hough accumulator votes")]
    vote_threshold: u32,

    #[arg(long, short, help = "Dump the normalized grayscale matrix to a raw binary file")]
    dump_grid: Option<PathBuf>,

    #[arg(long, short, help = "Output defect report as CSV")]
    output: Option<PathBuf>,

    #[arg(long, short = 'I', help = "Enable non-linear magnetic-dipole depth inversion (Levenberg-Marquardt GN)")]
    invert_depth: bool,

    #[arg(long, default_value_t = 1.0, help = "Sensor lift-off distance (mm) above OD for inversion model")]
    sensor_lift_mm: f32,
}

fn main() {
    let cli = Cli::parse();

    eprintln!("[mfl-audit] Opening MFL file: {:?}", cli.input);
    let stream = match MflStream::open(&cli.input) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[ERROR] Failed to open MFL stream: {e}");
            std::process::exit(1);
        }
    };

    let hdr = stream.header();
    eprintln!(
        "[mfl-audit] Header: v{} channels={} axes={} bits={} rate={}Hz OD={}mm WT={}mm spacing={:.2}°",
        hdr.version,
        hdr.num_channels,
        hdr.num_axes,
        hdr.sample_resolution_bits,
        hdr.frame_rate_hz,
        hdr.od_mm,
        hdr.wall_thickness_mm,
        hdr.sensor_spacing_deg,
    );

    let n_frames = stream.num_frames();
    eprintln!("[mfl-audit] Total frames: {n_frames}");

    if n_frames == 0 {
        eprintln!("[WARN] No frames found, exiting.");
        return;
    }

    eprintln!(
        "[mfl-audit] Building grid (axis={} vel={:.1} mm/s win={} sigma={:.1})...",
        cli.axis, cli.nominal_velocity, cli.median_window, cli.sigma_threshold
    );
    let grid = build_grid(
        &stream,
        cli.axis,
        cli.nominal_velocity,
        cli.median_window,
        cli.sigma_threshold,
    );
    eprintln!(
        "[mfl-audit] Grid dimensions: {} rows x {} cols",
        grid.rows, grid.cols
    );

    if let Some(ref path) = cli.dump_grid {
        let u8buf = grid.normalize_to_u8();
        match std::fs::write(path, &u8buf) {
            Ok(_) => eprintln!("[mfl-audit] Grid dumped to {:?}", path),
            Err(e) => eprintln!("[ERROR] Failed to dump grid: {e}"),
        }
    }

    eprintln!(
        "[mfl-audit] Running Hough detector (edge_thr={:.1} vote_thr={})...",
        cli.edge_threshold, cli.vote_threshold
    );
    let mut detector = HoughDetector::new();
    detector.edge_threshold = cli.edge_threshold;
    detector.vote_threshold = cli.vote_threshold;

    let defects = detector.detect(&grid);
    eprintln!("[mfl-audit] Detected {} defect region(s)", defects.len());

    for (i, d) in defects.iter().enumerate() {
        let class_str = match d.classification {
            DefectClass::Hyperbolic => "HYPERBOLIC",
            DefectClass::Elliptical => "ELLIPTICAL",
            DefectClass::Linear => "LINEAR",
            DefectClass::Unknown => "UNKNOWN",
        };
        eprintln!(
            "  [{}] center=({},{}) radius=({},{}) votes={} class={}",
            i + 1,
            d.center_row,
            d.center_col,
            d.radius_rows,
            d.radius_cols,
            d.peak_votes,
            class_str,
        );
    }

    if let Some(ref path) = cli.output {
        let mut csv = String::from("index,center_row,center_col,radius_rows,radius_cols,peak_votes,classification\n");
        for (i, d) in defects.iter().enumerate() {
            let class_str = match d.classification {
                DefectClass::Hyperbolic => "Hyperbolic",
                DefectClass::Elliptical => "Elliptical",
                DefectClass::Linear => "Linear",
                DefectClass::Unknown => "Unknown",
            };
            csv.push_str(&format!(
                "{},{},{},{},{},{},{}\n",
                i + 1,
                d.center_row,
                d.center_col,
                d.radius_rows,
                d.radius_cols,
                d.peak_votes,
                class_str,
            ));
        }
        match std::fs::write(path, &csv) {
            Ok(_) => eprintln!("[mfl-audit] Report written to {:?}", path),
            Err(e) => eprintln!("[ERROR] Failed to write report: {e}"),
        }
    }

    if cli.invert_depth && !defects.is_empty() {
        eprintln!("[mfl-audit] Extracting 3-axis defect cubes for depth inversion...");
        let cubes = extract_defect_cubes(
            &stream,
            &defects,
            cli.nominal_velocity,
            cli.median_window,
            cli.sigma_threshold,
            hdr.wall_thickness_mm,
        );
        eprintln!("[mfl-audit] Running magnetic-dipole inverse solver on {} cube(s) (wall={}mm lift={}mm)...",
            cubes.len(), hdr.wall_thickness_mm, cli.sensor_lift_mm);
        let depth_results = invert_all_cubes(
            &cubes,
            hdr.wall_thickness_mm as f64,
            cli.sensor_lift_mm as f64,
        );

        let total_dist_mm = stream
            .frame(stream.num_frames().saturating_sub(1))
            .map(|f| f.hdr.distance_mm as f64)
            .unwrap_or(stream.num_frames() as f64 * 0.5);

        let map_cfg = CorrosionMapConfig {
            wall_thickness_mm: hdr.wall_thickness_mm as f64,
            ..CorrosionMapConfig::default()
        };
        print_ascii_pipe_map(&depth_results, &defects, &map_cfg, total_dist_mm);
        print_defect_evolution_curve(&depth_results, total_dist_mm, 60);

        if let Some(ref path) = cli.output {
            let inv_path = path.with_extension("depth.csv");
            let mut csv = String::from(
                "region_id,distance_mm,angle_deg,length_mm,width_mm,depth_um,volume_mm3,severity,pct_wall,residual_rel,iterations,converged\n"
            );
            for r in &depth_results {
                let pct = if hdr.wall_thickness_mm > 0.0 {
                    r.geometry.depth_um * 1e-3 / hdr.wall_thickness_mm as f64 * 100.0
                } else { 0.0 };
                let sev_label = format!("{:?}", r.severity);
                csv.push_str(&format!(
                        "{},{},{},{},{},{},{},{},{},{},{},{}\n",
                        r.region_id,
                        r.geometry.pos_x_mm,
                        r.geometry.pos_y_mm,
                        r.geometry.length_mm,
                        r.geometry.width_mm,
                        r.geometry.depth_um,
                        r.geometry.volume_mm3(),
                        sev_label,
                        pct,
                        r.residual_rel,
                        r.iterations,
                        r.converged,
                ));
            }
            match std::fs::write(&inv_path, &csv) {
                Ok(_) => eprintln!("[mfl-audit] Depth inversion CSV written to {:?}", inv_path),
                Err(e) => eprintln!("[ERROR] Failed to write depth CSV: {e}"),
            }
        }
    }

    eprintln!("[mfl-audit] Done.");
}
