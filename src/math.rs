// reference: https://webglfundamentals.org
pub fn translation<'matrix>(tx: f32, ty: f32, tz: f32, dst: &'matrix mut [f32]) -> &'matrix [f32] {
  dst[ 0] = 1.0;
  dst[ 1] = 0.0;
  dst[ 2] = 0.0;
  dst[ 3] = 0.0;
  dst[ 4] = 0.0;
  dst[ 5] = 1.0;
  dst[ 6] = 0.0;
  dst[ 7] = 0.0;
  dst[ 8] = 0.0;
  dst[ 9] = 0.0;
  dst[10] = 1.0;
  dst[11] = 0.0;
  dst[12] = tx;
  dst[13] = ty;
  dst[14] = tz;
  dst[15] = 1.0;

  dst
}

fn rotate_x<'matrix>(angle: f32, dst: &'matrix mut [f32]) -> &'matrix [f32] {
  let m10 = dst[4];
  let m11 = dst[5];
  let m12 = dst[6];
  let m13 = dst[7];
  let m20 = dst[8];
  let m21 = dst[9];
  let m22 = dst[10];
  let m23 = dst[11];
  let c = angle.cos();
  let s = angle.sin();

  dst[4]  = c * m10 + s * m20;
  dst[5]  = c * m11 + s * m21;
  dst[6]  = c * m12 + s * m22;
  dst[7]  = c * m13 + s * m23;
  dst[8]  = c * m20 - s * m10;
  dst[9]  = c * m21 - s * m11;
  dst[10] = c * m22 - s * m12;
  dst[11] = c * m23 - s * m13;

  dst
}

fn rotate_y<'matrix>(angle: f32, dst: &'matrix mut [f32]) -> &'matrix [f32] {
  let m00 = dst[0 * 4 + 0];
  let m01 = dst[0 * 4 + 1];
  let m02 = dst[0 * 4 + 2];
  let m03 = dst[0 * 4 + 3];
  let m20 = dst[2 * 4 + 0];
  let m21 = dst[2 * 4 + 1];
  let m22 = dst[2 * 4 + 2];
  let m23 = dst[2 * 4 + 3];
  let c = angle.cos();
  let s = angle.sin();

  dst[ 0] = c * m00 - s * m20;
  dst[ 1] = c * m01 - s * m21;
  dst[ 2] = c * m02 - s * m22;
  dst[ 3] = c * m03 - s * m23;
  dst[ 8] = c * m20 + s * m00;
  dst[ 9] = c * m21 + s * m01;
  dst[10] = c * m22 + s * m02;
  dst[11] = c * m23 + s * m03;

  dst
}

fn rotate_z<'matrix>(angle: f32, dst: &'matrix mut [f32]) -> &'matrix [f32] {
  let m00 = dst[0 * 4 + 0];
  let m01 = dst[0 * 4 + 1];
  let m02 = dst[0 * 4 + 2];
  let m03 = dst[0 * 4 + 3];
  let m10 = dst[1 * 4 + 0];
  let m11 = dst[1 * 4 + 1];
  let m12 = dst[1 * 4 + 2];
  let m13 = dst[1 * 4 + 3];
  let c = angle.cos();
  let s = angle.sin();

  dst[ 0] = c * m00 + s * m10;
  dst[ 1] = c * m01 + s * m11;
  dst[ 2] = c * m02 + s * m12;
  dst[ 3] = c * m03 + s * m13;
  dst[ 4] = c * m10 - s * m00;
  dst[ 5] = c * m11 - s * m01;
  dst[ 6] = c * m12 - s * m02;
  dst[ 7] = c * m13 - s * m03;

  dst
}

pub fn v3_magnitude(x: f32, y: f32, z: f32) -> f32 {
  (x * x + y * y + z * z).sqrt()
}

pub fn v3_normalize((x, y, z): (f32, f32, f32)) -> (f32, f32, f32) {
  let mag = v3_magnitude(x, y, z);
  (x / mag, y / mag, z / mag)
}

pub fn v3_plane_distance(x: f32, y:f32, z: f32, px: f32, py: f32, pz: f32, nx: f32, ny: f32, nz: f32) -> f32 {
  (x - px) * nx + (y - py) * ny + (z - pz) * nz
}

pub const IDENTITY: [f32; 16] = [
  1.0, 0.0, 0.0, 0.0,
  0.0, 1.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.0,
  0.0, 0.0, 0.0, 1.0
];

pub fn rotate<'matrix>(v: &[f32], dst: &'matrix mut [f32]) -> &'matrix [f32] {
  let (ux, uy, uz) = v3_normalize((v[0], v[1], v[2]));
  let phi = uy.atan2(ux);
  let theta = ux.atan2(uz);
  let psi = uz.atan2(ux);
  rotate_z(phi, dst);
  rotate_y(theta, dst);
  rotate_x(psi, dst);
  dst
}
