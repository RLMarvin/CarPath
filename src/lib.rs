mod custom_util;
mod rlphysics;

use crate::custom_util::*;
use crate::rlphysics::*;
use glam::DVec2;

use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

#[pyclass(get_all)]
pub struct CarPath {
    pub player_circle: Vec<f64>,
    pub target_circle: Vec<f64>,
    pub player_tangent: Vec<f64>,
    pub target_tangent: Vec<f64>,
    pub total_distance: f64,
    pub total_time: f64,
    pub forwards: bool,
}

// impl Default for cParams {
//     fn default() -> cParams {
//         cParams {
//             player_circle: vec![0, 0],
//             target_circle: vec![0, 1],
//             player_tangent: vec![1, 0],
//             target_tangent: vec![1, 1],
//             total_distance: 1.0,
//             total_time: 1.0,
//             forwards: true,
//         }
//     }
// }

#[pyfunction]
pub fn tangent_point(
    circle: Vec<f64>,
    circle_radius: f64,
    point: Vec<f64>,
    angle_sign: f64,
) -> Vec<f64> {
    let mut result = vec![0.0; 2];

    let circle_distance = dist2d(&circle, &point) + 1e-9;
    let relative_angle = num_range(circle_radius / circle_distance, 1.0).acos();
    let point_angle = angle2d(&point, &circle);
    let tangent_angle = point_angle - relative_angle * custom_sign(angle_sign);

    result[0] = tangent_angle.cos() * circle_radius + circle[0];
    result[1] = tangent_angle.sin() * circle_radius + circle[1];

    return result;
}

#[pyfunction]
pub fn circles_tangent(
    c1: Vec<f64>,
    c1r: f64,
    c1s: f64,
    c2: Vec<f64>,
    c2r: f64,
    c2s: f64,
) -> Vec<Vec<f64>> {
    let out = custom_sign(c1s) != custom_sign(c2s);
    let vc1 = a2(&c1);
    let vc2 = a2(&c2);
    let c1t: DVec2;
    let c2t: DVec2;

    if c1r != c2r || !(out) {
        let pc2t = a2(&tangent_point(
            c2,
            c2r - c1r * custom_sign(out as u8 as f64),
            c1,
            c2s,
        ));
        c2t = set_dist_2d(
            vc2,
            pc2t,
            c2r * custom_sign((c2r > c1r || !(out)) as u8 as f64),
        );
        c1t = vc1 + c2t - pc2t;
    } else {
        c2t = set_dist_ang_2d(vc2, vc1, c2r, -PI / 2.0 * custom_sign(c2s));
        c1t = vc1 + c2t - vc2;
    }

    let mut result = vec![vec![0.0; 2], vec![0.0; 2]];

    result[0] = l2(c1t);
    result[1] = l2(c2t);

    return result;
}

#[pyfunction]
pub fn shootingPath(
    t_l: Vec<f64>,
    p_l: Vec<f64>,
    p_r: Vec<f64>,
    trd: f64,
    prd: f64,
    goal: Vec<f64>,
    pyv: f64,
    brd: Option<f64>,
    p_boost: Option<f64>,
) -> CarPath {
    let brd = brd.unwrap_or(92.0);
    let p_boost = p_boost.unwrap_or(100.0);

    let t_lb = set_dist(a3(&t_l), a3(&goal), -brd);

    let lt_l = local_space(a3(&t_l), a3(&p_l), a3(&p_r));
    let lt_lb = local_space(t_lb, a3(&p_l), a3(&p_r));

    let origin: Vec<f64> = vec![0.0, 55.0];

    let mut path: CarPath = CarPath {
        player_circle: vec![0.0; 2],
        target_circle: vec![0.0; 2],
        player_tangent: vec![0.0; 2],
        target_tangent: vec![0.0; 2],
        total_distance: 0.0,
        total_time: 0.0,
        forwards: true,
    };
    let mut b_first_loop = true;

    for t in 0..2 {
        let ltc_l = l3(set_dist_ang(
            lt_lb,
            lt_l,
            trd,
            PI / 2.0 * custom_sign(t as f64),
        ));
        for p in 0..2 {
            for f in 0..2 {
                let lpc_l = vec![
                    (prd * custom_sign(p as f64) * custom_sign(f as f64)),
                    55.0,
                    0.0,
                ];
                let tangents = circles_tangent(
                    lpc_l.clone(),
                    prd,
                    -custom_sign(p as f64),
                    ltc_l.clone(),
                    trd,
                    -custom_sign(t as f64),
                );
                let lpc_tl = tangents[0].clone();
                let ltc_tl = tangents[1].clone();
                let pca = relative_angle(&lpc_tl, &origin, &lpc_l).abs();
                let tca = range_360(
                    relative_angle(&ltc_tl, &l3(lt_lb), &ltc_l) * custom_sign(t as f64),
                    PI,
                );
                let td = pca * prd + dist2d(&lpc_tl, &ltc_tl) + tca * trd;
                let tt = min_travel_time(td, pyv * custom_sign(f as f64), p_boost * (f as f64));
                if b_first_loop || (tt < path.total_time) {
                    path.player_circle = lpc_l.clone();
                    path.target_circle = ltc_l.clone();
                    path.player_tangent = lpc_tl.clone();
                    path.target_tangent = ltc_tl.clone();
                    path.total_distance = td;
                    path.total_time = tt;
                    path.forwards = f != 0;
                    b_first_loop = false;
                }
            }
        }
    }

    return path;
}

#[pymodule]
fn car_path(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_class::<CarPath>()?;

    m.add_function(wrap_pyfunction!(tangent_point, m)?)?;
    m.add_function(wrap_pyfunction!(circles_tangent, m)?)?;
    m.add_function(wrap_pyfunction!(shootingPath, m)?)?;

    m.add_function(wrap_pyfunction!(max_travel_distance, m)?)?;
    m.add_function(wrap_pyfunction!(min_travel_time, m)?)?;

    Ok(())
}
