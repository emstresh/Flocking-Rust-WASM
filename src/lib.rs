mod utils;
mod math;

extern crate web_sys;
use wasm_bindgen::prelude::*;
use js_sys::Math;

use crate::math::{ v3_magnitude, v3_plane_distance, IDENTITY };

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
  k_pred_avoidance: f32,     // how strongly boids try to avoid predators
  boid_max_vel: f32,         // maximum boid velocity magnitude

  num_boids: usize,
  boids_pos: Vec<f32>,
  boids_vel: Vec<f32>,
  boids_col: Vec<u8>,
  boids_mat: Vec<f32>,
  boids_alive: Vec<bool>,
  boids_per_grid_cell: Vec<Vec<usize>>,

  k_eat: f32,                // how strongly predators want to eat a boid
  pred_max_vel: f32,         // maximum predator velocity magnitude

  num_predators: usize,
  pred_pos: Vec<f32>,
  pred_vel: Vec<f32>,
  pred_col: Vec<u8>,
  pred_mat: Vec<f32>,
  pred_num_eaten: Vec<u8>,
  pred_per_grid_cell: Vec<Vec<usize>>
}

fn rand_range(min: f32, max: f32) -> f32 {
  (max - min) * Math::random() as f32 + min
}

fn grid_cell_index(x: f32, y: f32, z: f32, num_chunks: usize, chunk_size: usize) -> usize {
  let ix = (x.floor() as usize % chunk_size) as usize;
  let iy = (y.floor() as usize % chunk_size) as usize;
  let iz = (z.floor() as usize % chunk_size) as usize;
  iz * num_chunks * num_chunks + iy * num_chunks + ix
}



#[wasm_bindgen]
impl Universe {
  pub fn new(dimension: usize, num_chunks: usize, num_boids: usize) -> Self {
    let chunk_size = dimension / num_chunks;

    // initialize boids
    let mut boids_pos = Vec::with_capacity(num_boids * 3);
    let mut boids_vel = Vec::with_capacity(num_boids * 3);
    let mut boids_col = Vec::with_capacity(num_boids * 3);
    let mut boids_mat = Vec::with_capacity(num_boids * 16); // 4x4 transformation matrix for each boid instance
    let mut boids_per_grid_cell = vec![Vec::new(); num_chunks * num_chunks * num_chunks];

    (0..num_boids).for_each(|i| {
      // random position
      (0..3).for_each(|_p| boids_pos.push(rand_range(0.0, dimension as f32)));

      // random velocity
      (0..3).for_each(|_v| boids_vel.push(rand_range(-10.0, 10.0)));

      // random blue-ish color
      boids_col.push(rand_range(64.0, 128.0) as u8);
      boids_col.push(rand_range(64.0, 128.0) as u8);
      boids_col.push(rand_range(192.0, 255.0) as u8);

      // transformation matrix
      (0..16).for_each(|m| boids_mat.push(IDENTITY[m]));
      let mat: &mut [f32] = &mut boids_mat[(i * 16)..(i * 16 + 16)];
      math::translation(boids_pos[i * 3], boids_pos[i * 3 + 1], boids_pos[i * 3 + 2], mat);
      math::rotate(&boids_vel[(i * 3)..(i * 3 + 3)], mat);

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

    // initialize predators (only 1 to start)
    let num_predators = 1;

    let mut pred_per_grid_cell = vec![Vec::new(); num_chunks * num_chunks * num_chunks];

    // num_eaten count controls how many sides the polygon representing
    // the predator will have (so no less than 3, even though that's weird)
    // guess this could also be just num_sides... but I like num_eaten
    let mut pred_num_eaten = Vec::with_capacity(num_predators);
    pred_num_eaten.push(3);

    // random position
    let mut pred_pos = Vec::with_capacity(num_predators * 3);
    (0..3).for_each(|_p| pred_pos.push(rand_range(0.0, dimension as f32)));

    // which grid cell did we generate the predator in?
    let idx = grid_cell_index(pred_pos[0], pred_pos[1], pred_pos[2], num_chunks, chunk_size);
    pred_per_grid_cell[idx].push(0);

    // random velocity
    let mut pred_vel = Vec::with_capacity(num_predators * 3);
    (0..3).for_each(|_v| pred_vel.push(rand_range(-10.0, 10.0)) );

    // random reddish color
    let mut pred_col = Vec::with_capacity(num_predators * 3);
    pred_col.push(rand_range(192.0, 255.0) as u8);
    pred_col.push(rand_range(64.0, 128.0) as u8);
    pred_col.push(rand_range(0.0, 64.0) as u8);

    // transformation matrices
    let mut pred_mat = Vec::with_capacity(num_predators * 16);
    (0..16).for_each(|m| pred_mat.push(IDENTITY[m]));
    let mat: &mut [f32] = &mut pred_mat[0..16];
    math::translation(pred_pos[0], pred_pos[1], pred_pos[2], mat);
    math::rotate(&pred_vel[0..3], mat);

    Self {
      dimension,
      num_chunks,
      chunk_size,
      k_obstacle_avoidance: 500.0,
      k_velocity_matching: 35.0,
      k_boid_avoidance: 1000.0,
      k_pred_avoidance: 100.0,
      boid_max_vel: 30.0,
      num_boids,
      boids_pos,
      boids_vel,
      boids_col,
      boids_mat,
      boids_alive: vec![true; num_boids],
      boids_per_grid_cell,
      k_eat: 250.0,
      pred_max_vel: 50.0,
      num_predators,
      pred_pos,
      pred_vel,
      pred_col,
      pred_mat,
      pred_num_eaten,
      pred_per_grid_cell
    }
  }

  pub fn tick(&mut self, delta: f32) {
    (0..self.num_boids).for_each(|i| self.update_boid(delta, i));
    (0..self.num_predators).for_each(|i| self.update_predator(delta, i));
  }

  pub fn num_boids(&self) -> usize {
    self.num_boids
  }

  pub fn boids_positions(&self) -> *const f32 {
    self.boids_pos.as_ptr()
  }

  pub fn boids_velocities(&self) -> *const f32 {
    self.boids_vel.as_ptr()
  }

  pub fn boids_matrices(&self) -> *const f32 {
    self.boids_mat.as_ptr()
  }

  pub fn boids_colors(&self) -> *const u8 {
    self.boids_col.as_ptr()
  }

  pub fn num_predators(&self) -> usize {
    self.num_predators
  }

  pub fn predators_positions(&self) -> *const f32 {
    self.pred_pos.as_ptr()
  }

  pub fn predators_matrices(&self) -> *const f32 {
    self.pred_mat.as_ptr()
  }

  pub fn predators_colors(&self) -> *const u8 {
    self.pred_col.as_ptr()
  }

  pub fn predators_eaten(&self) -> *const u8 {
    self.pred_num_eaten.as_ptr()
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
}

fn set_vec3<T>(arr: &mut Vec<T>, i: usize, x: T, y: T, z: T) {
  arr[i * 3] = x;
  arr[i * 3 + 1] = y;
  arr[i * 3 + 2] = z;
}

fn velocity_match(k: f32, my_pos: &[f32], my_vel: &[f32], other_pos: &[f32], other_vel: &[f32], acc: &mut [f32]) {
  // distance and inverse distance between the two boids
  let d = v3_magnitude(other_pos[0] - my_pos[0], other_pos[1] - my_pos[1], other_pos[2] - my_pos[2]);
  let d_inv = 1.0 / d;

  // adjust acceleration based on k factor and distance
  acc[0] = acc[0] + k * d_inv * (other_vel[0] - my_vel[0]);
  acc[1] = acc[1] + k * d_inv * (other_vel[1] - my_vel[1]);
  acc[2] = acc[2] + k * d_inv * (other_vel[2] - my_vel[2]);
}

fn avoid(k: f32, my_pos: &[f32], my_vel: &[f32], other_pos: &[f32], other_vel: &[f32], acc: &mut [f32]) {
  // distance and inverse distance between the two boids
  let d = v3_magnitude(other_pos[0] - my_pos[0], other_pos[1] - my_pos[1], other_pos[2] - my_pos[2]);
  let d_inv = 1.0 / d;

  // difference in velocity between the two boids
  let v_mag = v3_magnitude(other_vel[0] - my_vel[0], other_vel[1] - my_vel[1], other_vel[2] - my_vel[2]);
  let v_inv = 1.0 / v_mag;

  // adjust acceleration based on k factor and distance
  acc[0] = acc[0] - k * d_inv * (other_vel[0] - my_vel[0]) * v_inv;
  acc[1] = acc[1] - k * d_inv * (other_vel[1] - my_vel[1]) * v_inv;
  acc[2] = acc[2] - k * d_inv * (other_vel[2] - my_vel[2]) * v_inv;
}

fn avoid_bounds(k: f32, dimension: f32, my_pos: &[f32], acc: &mut [f32]) {
  let dx_pos = v3_plane_distance(my_pos[0], my_pos[1], my_pos[2], 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  let dx_neg = v3_plane_distance(my_pos[0], my_pos[1], my_pos[2], dimension, 0.0, 0.0, -1.0, 0.0, 0.0);
  let dy_pos = v3_plane_distance(my_pos[0], my_pos[1], my_pos[2], 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  let dy_neg = v3_plane_distance(my_pos[0], my_pos[1], my_pos[2], 0.0, dimension, 0.0, 0.0, -1.0, 0.0);
  let dz_pos = v3_plane_distance(my_pos[0], my_pos[1], my_pos[2], 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  let dz_neg = v3_plane_distance(my_pos[0], my_pos[1], my_pos[2], 0.0, 0.0, dimension, 0.0, 0.0, -1.0);
  acc[0] = acc[0] + k * (1.0 / dx_pos) * 1.0;
  acc[0] = acc[0] + k * (1.0 / dx_neg) * -1.0;
  acc[1] = acc[1] + k * (1.0 / dy_pos) * 1.0;
  acc[1] = acc[1] + k * (1.0 / dy_neg) * -1.0;
  acc[2] = acc[2] + k * (1.0 / dz_pos) * 1.0;
  acc[2] = acc[2] + k * (1.0 / dz_neg) * -1.0;
}

fn calculate_new_velocity(delta: f32, p: &[f32], v: &[f32], acc: &[f32], max_velocity: f32, dimension: f32) -> (f32, f32, f32) {
  // initial new velocity
  let (mut vx, mut vy, mut vz) = (v[0] + delta * acc[0], v[1] + delta * acc[1], v[2] + delta * acc[2]);

  // if we're heading out of bounds, turn around
  if (p[0] < 0.0 && vx < 0.0) || (p[0] > dimension && vx > 0.0) {
    vx = -vx
  } else if (p[1] < 0.0 && vy < 0.0) || (p[1] > dimension && vy > 0.0) {
    vy = -vy
  } else if (p[2] < 0.0 && vz < 0.0) || (p[2] > dimension && vz > 0.0) {
    vz = -vz
  }

  // magnitude of the new velocity
  let v_new_mag = v3_magnitude(vx, vy, vz);

  // cap velocity to a maximum
  if v_new_mag > max_velocity {
    let v_new_mag_inv = 1.0 / v_new_mag;
    vx = vx * v_new_mag_inv * max_velocity;
    vy = vy * v_new_mag_inv * max_velocity;
    vz = vz * v_new_mag_inv * max_velocity;
  }

  (vx, vy, vz)
}

impl Universe {
  fn update_predator(&mut self, delta: f32, i: usize) {
    // position and velocity of this predator
    let p = &self.pred_pos[(i * 3)..(i * 3 + 3)];
    let v = &self.pred_vel[(i * 3)..(i * 3 + 3)];

    // new acceleration value
    let acc: &mut [f32] = &mut vec![0.0; 3];

    // find the closest boid (check within our grid cell)
    let mut closest_boid_idx = 0;
    let mut closest_boid_distance = self.dimension as f32;
    let cell_index = grid_cell_index(p[0], p[1], p[2], self.num_chunks, self.chunk_size);
    for &idx in &self.boids_per_grid_cell[cell_index] {
    // for idx in 0..self.num_boids {
      if self.boids_alive[idx] {
        // position and velocity of this boid
        let pj = &self.boids_pos[(idx * 3)..(idx * 3 + 3)];
        let d = v3_magnitude(pj[0] - p[0], pj[1] - p[1], pj[2] - p[2]);
        if d < closest_boid_distance {
          closest_boid_distance = d;
          closest_boid_idx = idx;
        }
      }
    }

    // check the speed of the closest boid (new prey target) and follow it
    let d_inv = 1.0 / closest_boid_distance;
    let vb = &self.boids_pos[(closest_boid_idx * 3)..(closest_boid_idx * 3 + 3)];
    let v_mag = v3_magnitude(vb[0] - v[0], vb[1] - v[1], vb[2] - v[2]);
    let v_inv = 1.0 / v_mag;
    acc[0] = acc[0] + self.k_eat * d_inv * (vb[0] - v[0]) * v_inv;
    acc[1] = acc[1] + self.k_eat * d_inv * (vb[1] - v[1]) * v_inv;
    acc[2] = acc[2] + self.k_eat * d_inv * (vb[2] - v[2]) * v_inv;

    // avoid the other predators
    // for &idx in &self.pred_per_grid_cell[cell_index] {
    //   if idx != i {
    //     // position and velocity of this predator
    //     let pj = &self.pred_pos[(idx * 3)..(idx * 3 + 3)];
    //     let vj = &self.pred_vel[(idx * 3)..(idx * 3 + 3)];

    //     avoid(self.k_boid_avoidance, p, v, pj, vj, acc);
    //   }
    // }

    // make predators try to avoid cube boundaries
    avoid_bounds(self.k_obstacle_avoidance, self.dimension as f32, p, acc);

    // calculate new velocity and cap it to the maximum velocity
    let (vx, vy, vz) = calculate_new_velocity(delta, p, v, acc, self.pred_max_vel, self.dimension as f32);

    // calculate new position
    let (px, py, pz) = (p[0] + delta * v[0], p[1] + delta * v[1], p[2] + delta * v[2]);

    // update boids to new positions and velocities
    set_vec3(&mut self.pred_pos, i, px, py, pz);
    set_vec3(&mut self.pred_vel, i, vx, vy, vz);

    // move to new grid cell and remove from current one (if necessary)
    let new_cell_index = grid_cell_index(px, py, pz, self.num_chunks, self.chunk_size);
    if new_cell_index != cell_index && new_cell_index < self.dimension * self.dimension * self.dimension {
      self.pred_per_grid_cell[new_cell_index].push(i);
      // find this predator in the current grid cell and remove it
      let index_of_self = self.pred_per_grid_cell[cell_index].iter().position(|&r| r == i).unwrap();
      self.pred_per_grid_cell[cell_index].remove(index_of_self);
    }

    let mat: &mut [f32] = &mut self.pred_mat[(i * 16)..(i * 16 + 16)];
    math::translation(px, py, pz, mat);
    math::rotate(&self.pred_vel[(i * 3)..(i * 3 + 3)], mat);

    if self.pred_num_eaten[i] >= 10 {
      self.split_predator(i);
    }
  }

  fn update_boid(&mut self, delta: f32, i: usize) {
    if self.boids_alive[i] {

      // position and velocity of this boid
      let p = &self.boids_pos[(i * 3)..(i * 3 + 3)];
      let v = &self.boids_vel[(i * 3)..(i * 3 + 3)];

      // new acceleration value
      let acc: &mut [f32] = &mut vec![0.0; 3];

      // flock and velocity match with fellow boids in the cell
      let cell_index = grid_cell_index(p[0], p[1], p[2], self.num_chunks, self.chunk_size);
      for &idx in &self.boids_per_grid_cell[cell_index] {
        if idx != i && self.boids_alive[idx] {
          // position and velocity of this boid
          let pj = &self.boids_pos[(idx * 3)..(idx * 3 + 3)];
          let vj = &self.boids_vel[(idx * 3)..(idx * 3 + 3)];

          velocity_match(self.k_velocity_matching, p, v, pj, vj, acc);
          avoid(self.k_boid_avoidance, p, v, pj, vj, acc);
        }
      }

      // avoid predators
      for &idx in &self.pred_per_grid_cell[cell_index] {
      // for idx in 0..self.num_predators {
        let pj = &self.pred_pos[(idx * 3)..(idx * 3 + 3)];
        let vj = &self.pred_vel[(idx * 3)..(idx * 3 + 3)];
        let d = v3_magnitude(pj[0] - p[0], pj[1] - p[1], pj[2] - p[2]);

        avoid(self.k_pred_avoidance, p, v, pj, vj, acc);

        // if the predator is too close, the boid is eaten
        if d < self.boid_max_vel {
          // log!("CHOMP {}", i);
          self.boids_alive[i] = false;
          self.pred_num_eaten[idx] += 1;
        }
      }

      // make boids try to avoid cube boundaries
      avoid_bounds(self.k_obstacle_avoidance, self.dimension as f32, p, acc);

      // calculate new velocity and cap it to the maximum velocity
      let (vx, vy, vz) = calculate_new_velocity(delta, p, v, acc, self.boid_max_vel, self.dimension as f32);

      // calculate new boid position
      let (px, py, pz) = (p[0] + delta * v[0], p[1] + delta * v[1], p[2] + delta * v[2]);

      // update boids to new positions and velocities
      if self.boids_alive[i] {
        set_vec3(&mut self.boids_pos, i, px, py, pz);
        set_vec3(&mut self.boids_vel, i, vx, vy, vz);

        // move to new grid cell and remove from current one (if necessary)
        let new_cell_index = grid_cell_index(px, py, pz, self.num_chunks, self.chunk_size);
        if new_cell_index != cell_index && new_cell_index < self.dimension * self.dimension * self.dimension {
          self.boids_per_grid_cell[new_cell_index].push(i);
          // find this boid in the current grid cell and remove it
          let index_of_self = self.boids_per_grid_cell[cell_index].iter().position(|&r| r == i).unwrap();
          self.boids_per_grid_cell[cell_index].remove(index_of_self);
        }
      } else {
        set_vec3(&mut self.boids_pos, i, -1000.0, -1000.0, -1000.0);
        set_vec3(&mut self.boids_vel, i, 0.0, 0.0, 0.0);
      }

      let mat: &mut [f32] = &mut self.boids_mat[(i * 16)..(i * 16 + 16)];
      math::translation(px, py, pz, mat);
      math::rotate(&self.boids_vel[(i * 3)..(i * 3 + 3)], mat);

      // TODO: blood particles on boid death
    }
  }

  fn split_predator(&mut self, i: usize) {
    let new_idx = self.num_predators;
    self.num_predators += 1;

    self.pred_num_eaten[i] = 3;
    self.pred_num_eaten.push(3);

    let (px, py, pz) = (self.pred_pos[i * 3], self.pred_pos[i * 3 + 1], self.pred_pos[i * 3 + 2]);
    let (vx, vy, vz) = (self.pred_vel[i * 3], self.pred_vel[i * 3 + 1], self.pred_vel[i * 3 + 2]);

    // give the new predator the same position as the original one
    self.pred_pos.push(px);
    self.pred_pos.push(py);
    self.pred_pos.push(pz);

    // but with opposite velocity
    self.pred_vel.push(-vx);
    self.pred_vel.push(-vy);
    self.pred_vel.push(-vz);

    // and similar color
    self.pred_col.push(rand_range(192.0, 255.0) as u8);
    self.pred_col.push(rand_range(64.0, 128.0) as u8);
    self.pred_col.push(rand_range(0.0, 64.0) as u8);

    // which grid cell did we generate the predator in?
    let idx = grid_cell_index(px, py, pz, self.num_chunks, self.chunk_size);
    self.pred_per_grid_cell[idx].push(new_idx);

    // set new transformation matrix
    (0..16).for_each(|m| self.pred_mat.push(IDENTITY[m]));
    let mat: &mut [f32] = &mut self.pred_mat[(new_idx * 16)..(new_idx * 16 + 16)];
    math::translation(px, py, pz, mat);
    math::rotate(&self.pred_vel[(new_idx * 3)..(new_idx * 3 + 3)], mat);
  }
}