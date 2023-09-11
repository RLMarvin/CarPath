use glam::DVec2;
use glam::DVec3;

pub const PI: f64 = 3.141592653589793;

pub fn a2(v: &Vec<f64>) -> DVec2 {
    return DVec2::new(v[0], v[1]);
}

pub fn a3(v: &Vec<f64>) -> DVec3 {
    return DVec3::new(v[0], v[1], v[2]);
}

pub fn l2(v: DVec2) -> Vec<f64> {
    return vec![v[0], v[1]];
}

pub fn l3(v: DVec3) -> Vec<f64> {
    return vec![v[0], v[1], v[2]];
}

pub fn angle2_2d(a: &Vec<f64>, b: &Vec<f64>) -> f64 {
    return (a[0] - b[0]).atan2(a[1] - b[1]);
}

pub fn custom_sign(x: f64) -> f64 {
    if x > 0.0 {
        return 1.0;
    } else {
        return -1.0;
    }
}

pub fn dist2d(a: &Vec<f64>, b: &Vec<f64>) -> f64 {
    return ((a[0] - b[0]).powf(2.0) + (a[1] - b[1]).powf(2.0)).sqrt();
}

pub fn num_range(v: f64, r: f64) -> f64 {
    if v.abs() > r {
        return r * custom_sign(v);
    } else {
        return v;
    }
}

pub fn range_180(a: f64, pi: f64) -> f64 {
    let mut o: f64 = a;
    if a.abs() >= 2.0 * pi {
        o -= (a.abs() / (2.0 * pi)).floor() * 2.0 * pi * custom_sign(a);
    }
    if a.abs() > pi {
        o -= 2.0 * pi * custom_sign(a);
    }
    return o;
}

pub fn range_360(a: f64, pi: f64) -> f64 {
    return a - (a / (2.0 * pi)).floor() * 2.0 * pi;
}

pub fn relative_angle(a: &Vec<f64>, b: &Vec<f64>, o: &Vec<f64>) -> f64 {
    return range_180(angle2_2d(a, o) - angle2_2d(b, o), PI);
}

pub fn set_dist_2d(a: DVec2, b: DVec2, dist: f64) -> DVec2 {
    return (b - a).normalize() * dist + a;
}

pub fn set_dist(a: DVec3, b: DVec3, dist: f64) -> DVec3 {
    return (b - a).normalize() * dist + a;
}

pub fn rotate_2d(x: f64, y: f64, ang: f64) -> DVec2 {
    let cos_ang = ang.cos();
    let sin_ang = ang.sin();
    return DVec2::new(x * cos_ang - y * sin_ang, y * cos_ang + x * sin_ang);
}

pub fn set_dist_ang_2d(a: DVec2, b: DVec2, dist: f64, ang: f64) -> DVec2 {
    let c = (b - a).normalize() * dist;
    return rotate_2d(c[0], c[1], ang) + a;
}

pub fn set_dist_ang(a: DVec3, b: DVec3, dist: f64, ang: f64) -> DVec3 {
    let mut c = (b - a).normalize() * dist;
    let z = rotate_2d(c[0], c[1], ang);
    c[0] = z[0];
    c[1] = z[1];
    return c + a;
}

pub fn angle2d(a: &Vec<f64>, b: &Vec<f64>) -> f64 {
    return (a[1] - b[1]).atan2(a[0] - b[0]);
}

pub fn local_space(t_l: DVec3, o_l: DVec3, o_r: DVec3) -> DVec3 {
    let mut l = t_l - o_l;
    let pitch = -o_r[0];
    let yaw = -range_180(o_r[1] - PI / 2.0, PI);
    let roll = -o_r[2];
    let mut tmp = rotate_2d(l[0], l[1], yaw);
    l[0] = tmp[0];
    l[1] = tmp[1];
    tmp = rotate_2d(l[1], l[2], pitch);
    l[1] = tmp[0];
    l[2] = tmp[1];
    tmp = rotate_2d(l[0], l[2], roll);
    l[0] = tmp[0];
    l[2] = tmp[1];
    return l;
}
