import * as twgl from "twgl.js";

import { Universe } from "flocking-sim";
import { memory } from "flocking-sim/flocking_sim_bg";

import { fps } from "./js/stats";

const DIMENSION = 500;
const NUM_GRID_CHUNKS = 100;
const NUM_BOIDS = 500;

const universe = Universe.new(DIMENSION, NUM_GRID_CHUNKS, NUM_BOIDS);

const canvas = document.getElementById("c");
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

const gl = canvas.getContext("webgl");
if (!gl) {
  throw new Error("no webgl for you");
}
twgl.addExtensionsToContext(gl);
if (!gl.drawArraysInstanced || !gl.createVertexArray) {
  throw new Error("need drawArraysInstanced and createVertexArray");
}
const boundsProgramInfo = twgl.createProgramInfo(gl, ["bounds-vert", "color-frag"]);
const boidProgramInfo = twgl.createProgramInfo(gl, ["boid-vert", "color-frag"]);
const uniforms = {};

//
// BOUNDING CUBE
//
const bounds_geometry_arrays = {
  position: new Float32Array([
    0, 0, 0,
    DIMENSION, 0, 0,
    DIMENSION, DIMENSION, 0,
    0, DIMENSION, 0,
    0, 0, DIMENSION,
    DIMENSION, 0, DIMENSION,
    DIMENSION, DIMENSION, DIMENSION,
    0, DIMENSION, DIMENSION
  ]),
  indices: new Uint8Array([0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7]),
  color: new Uint8Array([
    255,   0,   0, 255,
    255, 128,   0, 255,
    128, 255,   0, 255,
      0, 255,   0, 255,
      0, 255, 128, 255,
      0, 128, 255, 255,
      0,   0, 255, 255,
    128,   0, 255, 255
  ])
};

function drawBounds() {
  const bufferInfo = twgl.createBufferInfoFromArrays(gl, bounds_geometry_arrays);
  const vertexArrayInfo = twgl.createVertexArrayInfo(gl, boundsProgramInfo, bufferInfo);

  gl.useProgram(boundsProgramInfo.program);
  twgl.setBuffersAndAttributes(gl, boundsProgramInfo, vertexArrayInfo);
  twgl.setUniforms(boundsProgramInfo, uniforms);
  twgl.drawBufferInfo(gl, vertexArrayInfo, gl.LINES);
}

//
// BOIDS
//
const numBoids = universe.num_boids();
const matrixPtr = universe.boids_matrices();
const matrices = new Float32Array(memory.buffer, matrixPtr, numBoids * 16);
const positionsPtr = universe.boids_positions();
const boidPositions =  new Float32Array(memory.buffer, positionsPtr, numBoids * 3);
const velocitiesPtr = universe.boids_velocities();
const boidVelocities =  new Float32Array(memory.buffer, velocitiesPtr, numBoids * 3);
const colorsPtr = universe.boids_colors();
const colors = new Uint8Array(memory.buffer, colorsPtr, numBoids * 3);

const boid_geometry_arrays = {
  position: [10,0,0,0,3,0,0,-3,0],
  indices: [0,1,2]
};

Object.assign(boid_geometry_arrays, {
  instanceWorld: {
    numComponents: 16,
    data: matrices,
    divisor: 1
  },
  instanceColor: {
    numComponents: 3,
    data: colors,
    divisor: 1
  }
});

function drawBoids() {
  const bufferInfo = twgl.createBufferInfoFromArrays(gl, boid_geometry_arrays);
  const vertexArrayInfo = twgl.createVertexArrayInfo(gl, boidProgramInfo, bufferInfo);

  gl.useProgram(boidProgramInfo.program);
  twgl.setBuffersAndAttributes(gl, boidProgramInfo, vertexArrayInfo);
  twgl.setUniforms(boidProgramInfo, uniforms);
  twgl.drawBufferInfo(gl, vertexArrayInfo, gl.TRIANGLES, vertexArrayInfo.numElements, 0, numBoids);
}

//
// PREDATORS
//
function drawPredators() {
  // TODO: new shader to get rid of instancing
  // TODO: can we only update some things on change?
  // TODO: why weren't my array views working?
  const numPredators = universe.num_predators();
  
  const eatenPtr = universe.predators_eaten();
  const eaten = new Uint8Array(memory.buffer, eatenPtr, numPredators);

  gl.useProgram(boidProgramInfo.program);
  for (let i = 0; i < numPredators; i++) {
    const predMatrixPtr = universe.predators_matrices(i);
    const predMatrices = new Float32Array(memory.buffer, predMatrixPtr, 16);
    const pColorsPtr = universe.predators_colors(i);
    const pColors = new Uint8Array(memory.buffer, pColorsPtr, 3);
    let arrays = ngon(eaten[i], 10);
    Object.assign(arrays, {
      instanceWorld: {
        numComponents: 16,
        data: predMatrices,
        divisor: 1
      },
      instanceColor: {
        numComponents: 3,
        data: pColors,
        divisor: 1
      }
    });
    const bufferInfo = twgl.createBufferInfoFromArrays(gl, arrays);
    const vertexArrayInfo = twgl.createVertexArrayInfo(gl, boidProgramInfo, bufferInfo);

    twgl.setBuffersAndAttributes(gl, boidProgramInfo, vertexArrayInfo);
    twgl.setUniforms(boidProgramInfo, uniforms);
    twgl.drawBufferInfo(gl, vertexArrayInfo, gl.TRIANGLES, vertexArrayInfo.numElements, 0, 1);
  }
}

//
// CAMERA
//
const fov = 45 * Math.PI / 180;
const aspect = gl.canvas.clientWidth / gl.canvas.clientHeight;
const zNear = 0.5;
const zFar = 2000;
const projection = twgl.m4.perspective(fov, aspect, zNear, zFar);
const radius = 250;
const target = [250, 250, 250];
const up = [0, 1, 0];

function updateCamera(time) {
  // const speed = time * 0.1;
  // const eye = [Math.cos(speed) * radius, 250, Math.sin(speed) * radius];
  const eye = [-250, 250, -250];

  const camera = twgl.m4.lookAt(eye, target, up);
  const view = twgl.m4.inverse(camera);
  uniforms.u_viewProjection = twgl.m4.multiply(projection, view);
  uniforms.u_viewInverse = camera;
}

//
// RENDER LOOP
//
let then = 0;
function render(time) {
  fps.render();

  time *= 0.001;
  let delta = time - then;

  universe.tick(delta);

  twgl.resizeCanvasToDisplaySize(gl.canvas);
  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height);

  gl.enable(gl.DEPTH_TEST);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  drawBounds();
  drawBoids();
  drawPredators();
  updateCamera(time);

  then = time;
  requestAnimationFrame(render);
}
requestAnimationFrame(render);

//
// create positions and indices for a regular polygon
function ngon(numSides, radius) {
  let geometry_array = {
    position: [0, 0, 0],
    indices: []
  };
  let dTheta = 2.0 * Math.PI / numSides;
  for (let i = 0; i < numSides; i++) {
    geometry_array.position.push(0);
    geometry_array.position.push(radius * Math.cos(i * dTheta));
    geometry_array.position.push(radius * Math.sin(i * dTheta));
    geometry_array.indices.push(0);
    geometry_array.indices.push(i + 1);
    if (i === numSides - 1) {
      geometry_array.indices.push(1);
    } else {
      geometry_array.indices.push(i + 2);
      // geometry_array.indices.push((i + 2) % numSides + 1);
    }
  }

  return geometry_array;
}
