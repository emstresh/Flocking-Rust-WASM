(window["webpackJsonp"] = window["webpackJsonp"] || []).push([[1],{

/***/ "../pkg/flocking_sim.js":
/*!******************************!*\
  !*** ../pkg/flocking_sim.js ***!
  \******************************/
/*! exports provided: Universe, __wbg_random_624219ee110d74d6, __wbindgen_throw */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"Universe\", function() { return Universe; });\n/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"__wbg_random_624219ee110d74d6\", function() { return __wbg_random_624219ee110d74d6; });\n/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"__wbindgen_throw\", function() { return __wbindgen_throw; });\n/* harmony import */ var _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! ./flocking_sim_bg.wasm */ \"../pkg/flocking_sim_bg.wasm\");\n\n\nconst lTextDecoder = typeof TextDecoder === 'undefined' ? __webpack_require__(/*! util */ \"./node_modules/util/util.js\").TextDecoder : TextDecoder;\n\nlet cachedTextDecoder = new lTextDecoder('utf-8', { ignoreBOM: true, fatal: true });\n\ncachedTextDecoder.decode();\n\nlet cachegetUint8Memory0 = null;\nfunction getUint8Memory0() {\n    if (cachegetUint8Memory0 === null || cachegetUint8Memory0.buffer !== _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"memory\"].buffer) {\n        cachegetUint8Memory0 = new Uint8Array(_flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"memory\"].buffer);\n    }\n    return cachegetUint8Memory0;\n}\n\nfunction getStringFromWasm0(ptr, len) {\n    return cachedTextDecoder.decode(getUint8Memory0().subarray(ptr, ptr + len));\n}\n\nfunction notDefined(what) { return () => { throw new Error(`${what} is not defined`); }; }\n/**\n*/\nclass Universe {\n\n    static __wrap(ptr) {\n        const obj = Object.create(Universe.prototype);\n        obj.ptr = ptr;\n\n        return obj;\n    }\n\n    free() {\n        const ptr = this.ptr;\n        this.ptr = 0;\n\n        _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"__wbg_universe_free\"](ptr);\n    }\n    /**\n    * @param {number} dimension\n    * @param {number} num_chunks\n    * @param {number} num_boids\n    * @returns {Universe}\n    */\n    static new(dimension, num_chunks, num_boids) {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_new\"](dimension, num_chunks, num_boids);\n        return Universe.__wrap(ret);\n    }\n    /**\n    * @param {number} delta\n    */\n    tick(delta) {\n        _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_tick\"](this.ptr, delta);\n    }\n    /**\n    * @returns {number}\n    */\n    num_boids() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_num_boids\"](this.ptr);\n        return ret >>> 0;\n    }\n    /**\n    * @returns {number}\n    */\n    boids_positions() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_boids_positions\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @returns {number}\n    */\n    boids_velocities() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_boids_velocities\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @returns {number}\n    */\n    boids_matrices() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_boids_matrices\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @returns {number}\n    */\n    boids_colors() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_boids_colors\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @returns {number}\n    */\n    num_predators() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_num_predators\"](this.ptr);\n        return ret >>> 0;\n    }\n    /**\n    * @returns {number}\n    */\n    predators_positions() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_predators_positions\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @returns {number}\n    */\n    predators_matrices() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_predators_matrices\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @returns {number}\n    */\n    predators_colors() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_predators_colors\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @returns {number}\n    */\n    predators_eaten() {\n        var ret = _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_predators_eaten\"](this.ptr);\n        return ret;\n    }\n    /**\n    * @param {number} val\n    */\n    set_k_obstacle_avoidance(val) {\n        _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_set_k_obstacle_avoidance\"](this.ptr, val);\n    }\n    /**\n    * @param {number} val\n    */\n    set_k_velocity_matching(val) {\n        _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_set_k_velocity_matching\"](this.ptr, val);\n    }\n    /**\n    * @param {number} val\n    */\n    set_k_boid_avoidance(val) {\n        _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_set_k_boid_avoidance\"](this.ptr, val);\n    }\n    /**\n    * @param {number} val\n    */\n    set_boid_max_vel(val) {\n        _flocking_sim_bg_wasm__WEBPACK_IMPORTED_MODULE_0__[\"universe_set_boid_max_vel\"](this.ptr, val);\n    }\n}\n\nconst __wbg_random_624219ee110d74d6 = typeof Math.random == 'function' ? Math.random : notDefined('Math.random');\n\nconst __wbindgen_throw = function(arg0, arg1) {\n    throw new Error(getStringFromWasm0(arg0, arg1));\n};\n\n\n\n//# sourceURL=webpack:///../pkg/flocking_sim.js?");

/***/ }),

/***/ "../pkg/flocking_sim_bg.wasm":
/*!***********************************!*\
  !*** ../pkg/flocking_sim_bg.wasm ***!
  \***********************************/
/*! exports provided: memory, __wbg_universe_free, universe_new, universe_tick, universe_num_boids, universe_boids_positions, universe_boids_velocities, universe_boids_matrices, universe_boids_colors, universe_num_predators, universe_predators_positions, universe_predators_matrices, universe_predators_colors, universe_predators_eaten, universe_set_k_obstacle_avoidance, universe_set_k_velocity_matching, universe_set_k_boid_avoidance, universe_set_boid_max_vel */
/***/ (function(module, exports, __webpack_require__) {

eval("\"use strict\";\n// Instantiate WebAssembly module\nvar wasmExports = __webpack_require__.w[module.i];\n__webpack_require__.r(exports);\n// export exports from WebAssembly module\nfor(var name in wasmExports) if(name != \"__webpack_init__\") exports[name] = wasmExports[name];\n// exec imports from WebAssembly module (for esm order)\n/* harmony import */ var m0 = __webpack_require__(/*! ./flocking_sim.js */ \"../pkg/flocking_sim.js\");\n\n\n// exec wasm module\nwasmExports[\"__webpack_init__\"]()\n\n//# sourceURL=webpack:///../pkg/flocking_sim_bg.wasm?");

/***/ }),

/***/ "./index.js":
/*!******************!*\
  !*** ./index.js ***!
  \******************/
/*! no exports provided */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony import */ var twgl_js__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! twgl.js */ \"./node_modules/twgl.js/dist/4.x/twgl-full.module.js\");\n/* harmony import */ var flocking_sim__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! flocking-sim */ \"../pkg/flocking_sim.js\");\n/* harmony import */ var flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! flocking-sim/flocking_sim_bg */ \"../pkg/flocking_sim_bg.wasm\");\n/* harmony import */ var _js_stats__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./js/stats */ \"./js/stats.js\");\n\n\n\n\n\n\n\nconst DIMENSION = 500;\nconst NUM_GRID_CHUNKS = 100;\nconst NUM_BOIDS = 5000;\n\nconst universe = flocking_sim__WEBPACK_IMPORTED_MODULE_1__[\"Universe\"].new(DIMENSION, NUM_GRID_CHUNKS, NUM_BOIDS);\n\nconst canvas = document.getElementById(\"c\");\ncanvas.width = window.innerWidth;\ncanvas.height = window.innerHeight;\n\nconst gl = canvas.getContext(\"webgl\");\nif (!gl) {\n  throw new Error(\"no webgl for you\");\n}\ntwgl_js__WEBPACK_IMPORTED_MODULE_0__[\"addExtensionsToContext\"](gl);\nif (!gl.drawArraysInstanced || !gl.createVertexArray) {\n  throw new Error(\"need drawArraysInstanced and createVertexArray\");\n}\nconst boundsProgramInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createProgramInfo\"](gl, [\"bounds-vert\", \"color-frag\"]);\nconst boidProgramInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createProgramInfo\"](gl, [\"boid-vert\", \"color-frag\"]);\nconst uniforms = {};\n\n//\n// BOUNDING CUBE\n//\nconst bounds_geometry_arrays = {\n  position: new Float32Array([\n    0, 0, 0,\n    DIMENSION, 0, 0,\n    DIMENSION, DIMENSION, 0,\n    0, DIMENSION, 0,\n    0, 0, DIMENSION,\n    DIMENSION, 0, DIMENSION,\n    DIMENSION, DIMENSION, DIMENSION,\n    0, DIMENSION, DIMENSION\n  ]),\n  indices: new Uint8Array([0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7]),\n  color: new Uint8Array([\n    255,   0,   0, 255,\n    255, 128,   0, 255,\n    128, 255,   0, 255,\n      0, 255,   0, 255,\n      0, 255, 128, 255,\n      0, 128, 255, 255,\n      0,   0, 255, 255,\n    128,   0, 255, 255\n  ])\n};\n\nfunction drawBounds() {\n  const bufferInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createBufferInfoFromArrays\"](gl, bounds_geometry_arrays);\n  const vertexArrayInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createVertexArrayInfo\"](gl, boundsProgramInfo, bufferInfo);\n\n  gl.useProgram(boundsProgramInfo.program);\n  twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"setBuffersAndAttributes\"](gl, boundsProgramInfo, vertexArrayInfo);\n  twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"setUniforms\"](boundsProgramInfo, uniforms);\n  twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"drawBufferInfo\"](gl, vertexArrayInfo, gl.LINES);\n}\n\n//\n// BOIDS\n//\nconst numBoids = universe.num_boids();\nconst matrixPtr = universe.boids_matrices();\nconst matrices = new Float32Array(flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__[\"memory\"].buffer, matrixPtr, numBoids * 16);\nconst positionsPtr = universe.boids_positions();\nconst boidPositions =  new Float32Array(flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__[\"memory\"].buffer, positionsPtr, numBoids * 3);\nconst velocitiesPtr = universe.boids_velocities();\nconst boidVelocities =  new Float32Array(flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__[\"memory\"].buffer, velocitiesPtr, numBoids * 3);\nconst colorsPtr = universe.boids_colors();\nconst colors = new Uint8Array(flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__[\"memory\"].buffer, colorsPtr, numBoids * 3);\n\nconst boid_geometry_arrays = {\n  position: [10,0,0,0,3,0,0,-3,0],\n  indices: [0,1,2]\n};\n\nObject.assign(boid_geometry_arrays, {\n  instanceWorld: {\n    numComponents: 16,\n    data: matrices,\n    divisor: 1\n  },\n  instanceColor: {\n    numComponents: 3,\n    data: colors,\n    divisor: 1\n  }\n});\n\nfunction drawBoids() {\n  const bufferInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createBufferInfoFromArrays\"](gl, boid_geometry_arrays);\n  const vertexArrayInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createVertexArrayInfo\"](gl, boidProgramInfo, bufferInfo);\n\n  twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"setBuffersAndAttributes\"](gl, boidProgramInfo, vertexArrayInfo);\n  twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"drawBufferInfo\"](gl, vertexArrayInfo, gl.TRIANGLES, vertexArrayInfo.numElements, 0, numBoids);\n}\n\n//\n// PREDATORS\n//\nconst PREDATOR_RADIUS = 10.0;\n\n// pregenerate geometry for the possible predator polygons\nconst predator_geometry_collection = {};\nfor (let i = 3; i <= 10; i++) { // predators range from 3 to 10 sides only\n  predator_geometry_collection[i] = ngon(i, PREDATOR_RADIUS);\n  Object.assign(predator_geometry_collection[i], {\n    instanceWorld: {\n      numComponents: 16,\n      data: [],\n      divisor: 1\n    },\n    instanceColor: {\n      numComponents: 3,\n      data: [],\n      divisor: 1\n    }\n  });\n}\n\nfunction drawPredators() {\n  const numPredators = universe.num_predators();\n\n  const eaten = new Uint8Array(flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__[\"memory\"].buffer, universe.predators_eaten(), numPredators);\n  const predatorDistribution = {};\n  for (let i = 0; i < numPredators; i++) {\n    let numEaten = eaten[i];\n    if (!predatorDistribution[numEaten]) predatorDistribution[numEaten] = [];\n    predatorDistribution[numEaten].push(i);\n  }\n\n  // instance predators by their size\n  const predMatrices = new Float32Array(flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__[\"memory\"].buffer, universe.predators_matrices(), numPredators * 16);\n  const pColors = new Uint8Array(flocking_sim_flocking_sim_bg__WEBPACK_IMPORTED_MODULE_2__[\"memory\"].buffer, universe.predators_colors(), numPredators * 3);\n  for (let size in predatorDistribution) {\n    let predatorIndices = predatorDistribution[size];\n    const matrices = new Float32Array(predatorIndices.length * 16);\n    const colors = new Uint8Array(predatorIndices.length * 3);\n    for (let i = 0; i < predatorIndices.length; i++) {\n      let idx = predatorIndices[i];\n      // fill in the matrices for the predators of this size\n      for (let j = 0; j < 16; j++) { matrices[i * 16 + j] = predMatrices[idx * 16 + j]; }\n      // fill in the colors for the predators of this size\n      for (let j = 0; j < 3; j++) { colors[i * 3 + j] = pColors[idx * 3 + j]; }\n    }\n    predator_geometry_collection[size].instanceWorld.data = matrices;\n    predator_geometry_collection[size].instanceColor.data = colors;\n  }\n\n  for (let size in predatorDistribution) {\n    let arrays = predator_geometry_collection[size];\n\n    const bufferInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createBufferInfoFromArrays\"](gl, arrays);\n    const vertexArrayInfo = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"createVertexArrayInfo\"](gl, boidProgramInfo, bufferInfo);\n\n    twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"setBuffersAndAttributes\"](gl, boidProgramInfo, vertexArrayInfo);\n    twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"drawBufferInfo\"](gl, vertexArrayInfo, gl.TRIANGLES, vertexArrayInfo.numElements, 0, predatorDistribution[size].length);\n  }\n}\n\n//\n// CAMERA\n//\nconst fov = 45 * Math.PI / 180;\nconst aspect = gl.canvas.clientWidth / gl.canvas.clientHeight;\nconst zNear = 0.5;\nconst zFar = 2000;\nconst projection = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"m4\"].perspective(fov, aspect, zNear, zFar);\nconst radius = 250;\nconst target = [250, 250, 250];\nconst up = [0, 1, 0];\n\nfunction updateCamera(time) {\n  // const speed = time * 0.1;\n  // const eye = [Math.cos(speed) * radius, 250, Math.sin(speed) * radius];\n  const eye = [-250, 250, -250];\n\n  const camera = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"m4\"].lookAt(eye, target, up);\n  const view = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"m4\"].inverse(camera);\n  uniforms.u_viewProjection = twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"m4\"].multiply(projection, view);\n  uniforms.u_viewInverse = camera;\n}\n\n//\n// RENDER LOOP\n//\nlet then = 0;\nfunction render(time) {\n  _js_stats__WEBPACK_IMPORTED_MODULE_3__[\"fps\"].render();\n\n  time *= 0.001;\n  let delta = time - then;\n\n  universe.tick(delta);\n\n  twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"resizeCanvasToDisplaySize\"](gl.canvas);\n  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height);\n\n  gl.enable(gl.DEPTH_TEST);\n  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);\n\n  drawBounds();\n\n  gl.useProgram(boidProgramInfo.program);\n  twgl_js__WEBPACK_IMPORTED_MODULE_0__[\"setUniforms\"](boidProgramInfo, uniforms);\n  drawBoids();\n  drawPredators();\n  \n  updateCamera(time);\n\n  then = time;\n  requestAnimationFrame(render);\n}\nrequestAnimationFrame(render);\n\n//\n// create positions and indices for a regular polygon\nfunction ngon(numSides, radius) {\n  let geometry_array = {\n    position: [0, 0, 0],\n    indices: []\n  };\n  let dTheta = 2.0 * Math.PI / numSides;\n  for (let i = 0; i < numSides; i++) {\n    geometry_array.position.push(0);\n    geometry_array.position.push(radius * Math.cos(i * dTheta));\n    geometry_array.position.push(radius * Math.sin(i * dTheta));\n    geometry_array.indices.push(0);\n    geometry_array.indices.push(i + 1);\n    if (i === numSides - 1) {\n      geometry_array.indices.push(1);\n    } else {\n      geometry_array.indices.push(i + 2);\n    }\n  }\n\n  return geometry_array;\n}\n\n\n//# sourceURL=webpack:///./index.js?");

/***/ }),

/***/ "./js/stats.js":
/*!*********************!*\
  !*** ./js/stats.js ***!
  \*********************/
/*! exports provided: fps */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"fps\", function() { return fps; });\nconst fps = new class {\n  constructor() {\n    this.fps = document.getElementById(\"fps\");\n    this.frames = [];\n    this.lastFrameTimeStamp = performance.now();\n  }\n\n  render() {\n    const now = performance.now();\n    const delta = now - this.lastFrameTimeStamp;\n    this.lastFrameTimeStamp = now;\n    const fps = 1 / delta * 1000;\n\n    this.frames.push(fps);\n    if (this.frames.length > 100) {\n      this.frames.shift();\n    }\n\n    let min = Infinity;\n    let max = -Infinity;\n    let sum = 0;\n    for (let i = 0; i < this.frames.length; i++) {\n      sum += this.frames[i];\n      min = Math.min(this.frames[i], min);\n      max = Math.max(this.frames[i], max);\n    }\n    let mean = sum / this.frames.length;\n\n    this.fps.textContent = `\nFrames per Second:\n         latest = ${Math.round(fps)}\navg of last 100 = ${Math.round(mean)}\nmin of last 100 = ${Math.round(min)}\nmax of last 100 = ${Math.round(max)}\n`.trim();\n  }\n}\n\n\n\n//# sourceURL=webpack:///./js/stats.js?");

/***/ })

}]);