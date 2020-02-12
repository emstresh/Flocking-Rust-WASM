import * as twgl from "twgl.js";

import { Universe } from "flocking-sim";
import { memory } from "flocking-sim/flocking_sim_bg";

import { fps } from "./js/stats";

const DIMENSION = 500;
const NUM_GRID_CHUNKS = 100;
const NUM_BOIDS = 2000;

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

  twgl.setBuffersAndAttributes(gl, boidProgramInfo, vertexArrayInfo);
  twgl.drawBufferInfo(gl, vertexArrayInfo, gl.TRIANGLES, vertexArrayInfo.numElements, 0, numBoids);
}

//
// PREDATORS
//
const PREDATOR_RADIUS = 10.0;

// pregenerate geometry for the possible predator polygons
const predator_geometry_collection = {};
for (let i = 3; i <= 10; i++) { // predators range from 3 to 10 sides only
  predator_geometry_collection[i] = ngon(i, PREDATOR_RADIUS);
  Object.assign(predator_geometry_collection[i], {
    instanceWorld: {
      numComponents: 16,
      data: [],
      divisor: 1
    },
    instanceColor: {
      numComponents: 3,
      data: [],
      divisor: 1
    }
  });
}

function drawPredators() {
  const numPredators = universe.num_predators();

  const eaten = new Uint8Array(memory.buffer, universe.predators_eaten(), numPredators);
  const predatorDistribution = {};
  for (let i = 0; i < numPredators; i++) {
    let numEaten = eaten[i];
    if (!predatorDistribution[numEaten]) predatorDistribution[numEaten] = [];
    predatorDistribution[numEaten].push(i);
  }

  // instance predators by their size
  const predMatrices = new Float32Array(memory.buffer, universe.predators_matrices(), numPredators * 16);
  const pColors = new Uint8Array(memory.buffer, universe.predators_colors(), numPredators * 3);
  for (let size in predatorDistribution) {
    let predatorIndices = predatorDistribution[size];
    const matrices = new Float32Array(predatorIndices.length * 16);
    const colors = new Uint8Array(predatorIndices.length * 3);
    for (let i = 0; i < predatorIndices.length; i++) {
      let idx = predatorIndices[i];
      // fill in the matrices for the predators of this size
      for (let j = 0; j < 16; j++) { matrices[i * 16 + j] = predMatrices[idx * 16 + j]; }
      // fill in the colors for the predators of this size
      for (let j = 0; j < 3; j++) { colors[i * 3 + j] = pColors[idx * 3 + j]; }
    }
    predator_geometry_collection[size].instanceWorld.data = matrices;
    predator_geometry_collection[size].instanceColor.data = colors;
  }

  for (let size in predatorDistribution) {
    let arrays = predator_geometry_collection[size];

    const bufferInfo = twgl.createBufferInfoFromArrays(gl, arrays);
    const vertexArrayInfo = twgl.createVertexArrayInfo(gl, boidProgramInfo, bufferInfo);

    twgl.setBuffersAndAttributes(gl, boidProgramInfo, vertexArrayInfo);
    twgl.drawBufferInfo(gl, vertexArrayInfo, gl.TRIANGLES, vertexArrayInfo.numElements, 0, predatorDistribution[size].length);
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

  gl.useProgram(boidProgramInfo.program);
  twgl.setUniforms(boidProgramInfo, uniforms);
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
    }
  }

  return geometry_array;
}
