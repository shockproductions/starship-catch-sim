# CATCH//SIM — Starship tower-catch simulator

**▶ Play it in your browser: https://shockproductions.github.io/starship-catch-sim/** (desktop and phone)

A live simulation of a Starship upper stage returning to the launch site. It glides belly-first, relights its Raptors, flips, flies a hoverslam into the tower and is caught by the chopsticks. Everything runs in real time in the browser, so you can change any parameter in the middle of a run and watch the flight computer react. Nothing is precomputed.

## What's simulated

- **6-DOF rigid-body physics** at 500 Hz: quaternion attitude, Euler's equations, propellant burn-off.
- **Basic aerodynamics:** crossflow drag model for the body. Each of the four flaps is a flat plate, with drag and moments that depend on how far it's deployed. The centre of mass sits behind the centre of pressure, so the ship is unstable in pitch and the flaps actively stabilise it.
- **Raptor engines:** stochastic spin-up time, start-failure probability, per-engine thrust dispersion, deep throttling, a gimbal with rate limits, and in-flight failures.
- **Environment:** wind profile with altitude and directional veer, Gauss-Markov turbulence, discrete gusts, and an imperfect wind forecast that the controller has to rely on.
- **Sensors and navigation:** IMU propagation with bias random-walk, GNSS at 10 Hz, radar altimeter, and attitude-knowledge error. The controller only ever sees these estimates, never the true state.
- **Tower:** chopstick arms with closing dynamics, contact forces between the arms and the hull, and catch pins that load the rails, measured by load cells.

## The flight computer (GNC)

| Phase | Guidance / control |
|---|---|
| Belly-flop | E-guidance to the relight gate. Steering is done by banking and pitching the ship. The aim point leads to account for the flip's braking kick and leans upwind using the forecast. |
| Ignition / flip | 3 engines; flaps pitch the nose up, then gimbal and flaps rotate the ship to vertical. |
| Landing burn | Continuous **hoverslam**: vertical constant-deceleration E-guidance with no hovering. It aims just offshore of the tower until the divert gate. |
| Tower divert | On GO, the target moves into the arms. Lateral control uses a braking-distance-aware law with feed-forward, and the vertical timing is tied to how long the lateral move needs. |
| Catch | Single engine, arms closing, slow sink onto the rails. Commit point 6 m above the rails; engines cut on pin contact. |

**Attitude control** uses a rate-limited PD controller whose gain scales with available thrust. Torque is split between the engine gimbal, a regularised least-squares allocation across the four flaps, and RCS for whatever is left. The thrust direction is steered, not just the body axis, so steady gimbal trim doesn't push the ship sideways.

**Fail-safes:**

- **Pre-relight catch poll:** checks weather, navigation, tower status and propellant.
- **Adaptive relight altitude:** raises the relight gate when the one you set is too low.
- **Engine checks:** a relight count check, recovery from an engine-out, and an emergency relight of a standby engine.
- **Divert gate:** one GO/NO-GO decision covering engines, navigation, winds, tower, divert time and propellant.
- **Tower protection:** a predictive keep-out monitor and lateral-overshoot monitor, plus a tower repulsion field. Abort paths climb over the tower only when it's in the way.
- **Bingo and commit logic:** a propellant bingo, an immediate abort on GNSS loss or a tower fault, and a commit point.

Every abort ends in a hoverslam into the water offshore.

The propellant budget is tight on purpose. With 11 t in the header tanks, a nominal catch finishes with about 1.5 t left.

## Things to try

- **Monte-Carlo:** enable *Auto-run next seed* and set the speed to 16–32×. Success rates and landing points build up in the run history and on the map.
- **Engine start timing:** raise *Spin-up time σ*. The flip-end point scatters, the divert gate starts saying NO-GO, and diverts go offshore.
- **Relight altitude:** drop it to 300 m. The adaptive fail-safe saves you; switch it off and you lose the vehicle.
- **Faults:** inject a Raptor failure, a jammed flap, GPS loss or an arm fault mid-approach.
- **Weather:** set the wind above the catch limit and watch poll 1 retarget the glide offshore.

Keyboard: `Space` pause, `R` restart seed, `N` next seed, `+`/`-` speed, `1`–`5` cameras.
URL options: `?seed=7&ff=120&cam=pad` (`ff` = fast-forward seconds).

## Run locally

Open `index.html` in a browser, or on Windows double-click `run_starship_catch_sim.bat`. three.js loads from a CDN, so you need an internet connection the first time.

`sim_core.js` has no DOM dependencies and also runs headless in Node (`require('./sim_core.js')`), which is useful for batch Monte-Carlo runs.

## Disclaimer

This is an independent fan/educational simulation. Vehicle and site parameters are plausible estimates, not SpaceX data, and it is not affiliated with SpaceX.

MIT licensed.
