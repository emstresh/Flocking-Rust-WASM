mod utils;

extern crate web_sys;
use wasm_bindgen::prelude::*;
// use cgmath::Vector3;
use js_sys::Math;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

// A macro to provide `println!(..)`-style syntax for `console.log` logging.
macro_rules! log {
  ( $( $t:tt )* ) => {
    web_sys::console::log_1(&format!( $( $t )* ).into());
  }
}

#[wasm_bindgen]
extern {
  fn alert(s: &str);
}

#[wasm_bindgen]
pub struct Universe {
  dimension: usize,
  num_chunks: usize,
  chunk_size: usize,

  k_obstacle_avoidance: f32, // how strongly boids try to avoid obstacles (like cube boundaries)
  k_velocity_matching: f32,  // how strongly boids try to match velocity with their fellow boids
  k_boid_avoidance: f32,     // how strongly boids try to avoid running into each other
  boid_max_vel: f32,         // maximum boid velocity magnitude

  num_boids: usize,
  boids_pos: Vec<f32>,
  boids_vel: Vec<f32>,
  boids_col: Vec<u8>,
  boids_alive: Vec<bool>,
  boids_per_grid_cell: Vec<Vec<usize>>
}

fn uniform_random(min: f32, max: f32) -> f32 {
  (max - min) * Math::random() as f32 + min
}

fn grid_cell_index(x: f32, y: f32, z: f32, num_chunks: usize, chunk_size: usize) -> usize {
  let ix = (x.floor() as usize % chunk_size) as usize;
  let iy = (y.floor() as usize % chunk_size) as usize;
  let iz = (z.floor() as usize % chunk_size) as usize;
  iz * num_chunks * num_chunks + iy * num_chunks + ix
}

fn v3_magnitude(x: f32, y: f32, z: f32) -> f32 {
  (x * x + y * y + z * z).sqrt()
}

fn v3_plane_distance(x: f32, y:f32, z: f32, px: f32, py: f32, pz: f32, nx: f32, ny: f32, nz: f32) -> f32 {
  (x - px) * nx + (y - py) * ny + (z - pz) * nz
}

#[wasm_bindgen]
impl Universe {
  pub fn new(dimension: usize, num_chunks: usize, num_boids: usize) -> Self {
    let chunk_size = dimension / num_chunks;
    let half_dimension = dimension as f32 / 2.0;

    let mut boids_pos = Vec::with_capacity(num_boids * 3);
    let mut boids_vel = Vec::with_capacity(num_boids * 3);
    let mut boids_col = Vec::with_capacity(num_boids * 4);
    let mut boids_per_grid_cell = vec![Vec::new(); num_chunks * num_chunks * num_chunks];

    (0..num_boids).for_each(|i| {
      // random position
      (0..3).for_each(|_p| boids_pos.push(uniform_random(-half_dimension, half_dimension as f32)) );

      // random velocity
      (0..3).for_each(|_v| boids_vel.push(uniform_random(-10.0, 10.0)) );

      // random color
      (0..4).for_each(|_c| boids_col.push(uniform_random(0.0, 255.0) as u8) );

      // which grid cell did we generate the boid in?
      let idx = grid_cell_index(
        boids_pos[i * 3],
        boids_pos[i * 3 + 1],
        boids_pos[i * 3 + 2],
        num_chunks,
        chunk_size
      );
      boids_per_grid_cell[idx].push(i);
    });

    Self {
      dimension,
      num_chunks,
      chunk_size,
      k_obstacle_avoidance: 500.0,
      k_velocity_matching: 35.0,
      k_boid_avoidance: 1000.0,
      boid_max_vel: 30.0,
      num_boids,
      boids_pos,
      boids_vel,
      boids_col,
      boids_alive: vec![true; num_boids],
      boids_per_grid_cell
    }
  }

  pub fn tick(&mut self, delta: f32) {
    (0..self.num_boids).for_each(|i| self.update_boid(delta, i));
  }

  fn update_boid(&mut self, delta: f32, i: usize) {
    if self.boids_alive[i] {
      let (xi, yi, zi) = (i * 3, i * 3 + 1, i * 3 + 2);

      // position and velocity of this boid
      let (pix, piy, piz) = (self.boids_pos[xi], self.boids_pos[yi], self.boids_pos[zi]);
      let (vix, viy, viz) = (self.boids_vel[xi], self.boids_vel[yi], self.boids_vel[zi]);

      // new acceleration value
      let (mut ax, mut ay, mut az) = (0.0, 0.0, 0.0);

      let half_dimension = self.dimension as f32 / 2.0;
      
      // flock and velocity match with fellow boids in the cell
      let cell_index = grid_cell_index(pix, piy, piz, self.num_chunks, self.chunk_size);
      for &idx in &self.boids_per_grid_cell[cell_index] {
        if idx != i {
          let (xj, yj, zj) = (idx * 3, idx * 3 + 1, idx * 3 + 2);

          // position and velocity of this boid
          let (pjx, pjy, pjz) = (self.boids_pos[xj], self.boids_pos[yj], self.boids_pos[zj]);
          let (vjx, vjy, vjz) = (self.boids_vel[xj], self.boids_vel[yj], self.boids_vel[zj]);

          // distance and inverse distance between the two boids
          let p_mag = v3_magnitude(pjx - pix, pjy - piy, pjz - piz);
          let p_mag_inv = 1.0 / p_mag;

          // velocity difference between the two boids
          let v_mag = v3_magnitude(vjx - vix, vjy - viy, vjz - viz);
          let v_mag_inv = 1.0 / v_mag;

          ax = ax + self.k_velocity_matching * p_mag_inv * (vjx - vix) + (-self.k_boid_avoidance * p_mag_inv) * (vjx - vix) * v_mag_inv;
          ay = ay + self.k_velocity_matching * p_mag_inv * (vjy - viy) + (-self.k_boid_avoidance * p_mag_inv) * (vjy - viy) * v_mag_inv;
          az = az + self.k_velocity_matching * p_mag_inv * (vjz - viz) + (-self.k_boid_avoidance * p_mag_inv) * (vjz - viz) * v_mag_inv;
        }
      }

      // TODO: avoid predators or be eaten

      // make boids try to avoid cube boundaries
      let dx_pos = v3_plane_distance(pix, piy, piz, -half_dimension, 0.0, 0.0, 1.0, 0.0, 0.0);
      ax = ax + self.k_obstacle_avoidance * (1.0 / dx_pos) * 1.0;
      let dx_neg = v3_plane_distance(pix, piy, piz, half_dimension, 0.0, 0.0, -1.0, 0.0, 0.0);
      ax = ax + self.k_obstacle_avoidance * (1.0 / dx_neg) * -1.0;
      let dy_pos = v3_plane_distance(pix, piy, piz, 0.0, -half_dimension, 0.0, 0.0, 1.0, 0.0);
      ay = ay + self.k_obstacle_avoidance * (1.0 / dy_pos) * 1.0;
      let dy_neg = v3_plane_distance(pix, piy, piz, 0.0, half_dimension, 0.0, 0.0, -1.0, 0.0);
      ay = ay + self.k_obstacle_avoidance * (1.0 / dy_neg) * -1.0;
      let dz_pos = v3_plane_distance(pix, piy, piz, 0.0, 0.0, -half_dimension, 0.0, 0.0, 1.0);
      az = az + self.k_obstacle_avoidance * (1.0 / dz_pos) * 1.0;
      let dz_neg = v3_plane_distance(pix, piy, piz, 0.0, 0.0, half_dimension, 0.0, 0.0, -1.0);
      az = az + self.k_obstacle_avoidance * (1.0 / dz_neg) * -1.0;

      // initial new velocity
      let (mut vx_new, mut vy_new, mut vz_new) = (vix + delta * ax, viy + delta * ay, viz + delta * az);

      // magnitude and inverse magnitude of the new velocity
      let mut v_new_mag = v3_magnitude(vx_new, vy_new, vz_new);
      let mut v_new_mag_inv = 1.0 / v_new_mag;

      // cap boid velocity to a maximum
      if v_new_mag > self.boid_max_vel {
        vx_new = vx_new * v_new_mag_inv * self.boid_max_vel;
        vy_new = vy_new * v_new_mag_inv * self.boid_max_vel;
        vz_new = vz_new * v_new_mag_inv * self.boid_max_vel;

        v_new_mag = self.boid_max_vel;
        v_new_mag_inv = 1.0 / v_new_mag;
      }

      // if boid is going to leave cube dimensions,
      // reverse velocity and normalize
      let (px_new, py_new, pz_new) = (pix + delta * vix, piy + delta * viy, piz + delta * viz);

      // if px_new < -half_dimension || px_new > half_dimension ||
      //    py_new < -half_dimension || py_new > half_dimension ||
      //    pz_new < -half_dimension || pz_new > half_dimension {
      //   vx_new = -vx_new * v_new_mag_inv;
      //   vy_new = -vy_new * v_new_mag_inv;
      //   vz_new = -vz_new * v_new_mag_inv;
      // }

      // update boids to new positions and velocities
      self.boids_pos[xi] = px_new;
      self.boids_pos[yi] = py_new;
      self.boids_pos[zi] = pz_new;

      self.boids_vel[xi] = vx_new;
      self.boids_vel[yi] = vy_new;
      self.boids_vel[zi] = vz_new;

      // TODO: blood particles
    }
  }

  pub fn num_boids(&self) -> usize {
    self.num_boids
  }

  pub fn boid_colors(&self) -> *const u8 {
    self.boids_col.as_ptr()
  }

  pub fn set_k_obstacle_avoidance(&mut self, val: f32) {
    self.k_obstacle_avoidance = val;
  }

  pub fn set_k_velocity_matching(&mut self, val: f32) {
    self.k_velocity_matching = val;
  }

  pub fn set_k_boid_avoidance(&mut self, val: f32) {
    self.k_boid_avoidance = val;
  }

  pub fn set_boid_max_vel(&mut self, val: f32) {
    self.boid_max_vel = val;
  }

  pub fn boid_triangles(&self) -> *const f32 {
    let mut triangles = Vec::with_capacity(self.num_boids * 3);
    for i in 0..self.num_boids {
      let xi = i * 3;
      let yi = xi + 1;
      let zi = yi + 1;
      // first vertex (facing velocity direction)
      triangles.push(self.boids_pos[xi] + self.boids_vel[xi]);
      triangles.push(self.boids_pos[yi] + self.boids_vel[yi]);
      triangles.push(self.boids_pos[zi] + self.boids_vel[zi]);
      // second vertex
      triangles.push(self.boids_pos[xi]);
      triangles.push(self.boids_pos[yi] + 1.0);
      triangles.push(self.boids_pos[zi]);
      // third vertex
      triangles.push(self.boids_pos[xi]);
      triangles.push(self.boids_pos[yi] - 1.0);
      triangles.push(self.boids_pos[zi]);
    }
    triangles.as_ptr()
  }
}