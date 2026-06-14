use std::path::PathBuf;

use clap::Parser;

use mfl_audit::stream_parser::MflStream;
use mfl_audit::grid::build_grid;
use mfl_audit::hough::{HoughDetector, DefectClass};

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

    eprintln!("[mfl-audit] Done.");
}
