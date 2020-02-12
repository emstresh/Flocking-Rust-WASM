[demo](https://emstresh.github.io/Flocking-Rust-WASM/)

Test Rust/WASM project, reimplementing an old flocking sim from grad school (C++/OpenGL) in Rust/WASM and WebGL.

Ref: https://webglfundamentals.org

Rules:
- Boids will velocity match with one another and try not to collide. They will avoid predators.
- Predators chase boids and try to eat them. If a predator eats a boid, it's polygon shape gains 1 side. When a predator has 10 sides, it splits into 2 3-sided predators.

To run:

```
wasm-pack build
```
in root directory, and
```
npm i
```
in docs/ directory. Then
```
npm run start
```
and site will be at localhost:8080
