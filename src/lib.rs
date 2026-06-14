pub mod mfl_format;
pub mod stream_parser;
pub mod signal_proc;
pub mod grid;
pub mod hough;
pub mod ringbuf;
pub mod dipole;
pub mod defect_extract;
pub mod depth_inversion;
pub mod ascii_plot;

pub use stream_parser::MflStream;
pub use grid::{GridMatrix, build_grid};
pub use hough::HoughDetector;
pub use dipole::{DipoleForwardModel, DefectGeometry, Vec3};
pub use defect_extract::extract_defect_cubes;
pub use depth_inversion::{invert_all_cubes, InversionResult, DepthSeverity, classify_depth};
pub use ascii_plot::{print_ascii_pipe_map, print_defect_evolution_curve, CorrosionMapConfig};
