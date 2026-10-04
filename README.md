# procedural-rocket

Build **multi-stage launch vehicles** and **fly them** — in **pure Python**, no dependencies.

## What it does

1. **Stack a rocket** (`stages.py`): pile up stages and strap on side
   boosters, each with its own thrust, burn time, dry/fuel mass and ISP.
   Exports the whole vehicle as one `aurora9.obj` mesh (stages, interstages,
   nose cone, engine bells, fins) — open it in Blender or 3dviewer.net.
2. **Launch it** (`launch.py`): a 2-DOF trajectory simulator — thrust,
   gravity, atmospheric drag, a pitch-program gravity turn, staging events,
   booster separation, max-Q detection and apogee. Prints a mini
   **mission-control readout** with ASCII altitude/velocity plots.
3. **The classic** (`rocket.py`): the original single-rocket OBJ generator —
   still works exactly as before.

## Run it

```bash
python3 stages.py   # build the Aurora-9 demo rocket -> aurora9.obj
python3 launch.py   # fly it -> mission control readout
python3 rocket.py   # original single-rocket generator -> rocket.obj
```

## The demo mission: Aurora-9

2 stages + 2 side boosters, 836,500 kg on the pad, 12,000 kN of thrust
(liftoff TWR 1.46):

```
  EVENT LOG
    T+   0.0s  LIFTOFF            alt     0.0 km   vel      0 m/s
    T+  60.0s  MAX-Q              alt    11.3 km   vel    445 m/s
    T+  62.0s  BOOSTER SEP        alt    12.1 km   vel    468 m/s
    T+ 152.0s  MECO-1 / STAGE SEP / STAGE-2 IGNITION
    T+ 472.0s  MECO-2 / SECO      alt   507.9 km   vel   4623 m/s
    T+ 614.0s  APOGEE             alt   590.2 km
```

Plus a per-stage summary (propellant used, delta-v via Tsiolkovsky, max
TWR) and ASCII plots of altitude-vs-time and velocity-vs-time, with every
event marked on the curve.

## How it works

- **Builder**: each stage/booster is a spec object (`thrust_kn`, `burn_s`,
  `dry_kg`, `fuel_kg`, `isp_s` …). The mesh builder stacks cylinders ring
  by ring with `cos`/`sin`, adds nose cones, engine bells and fins, and
  writes one Wavefront OBJ with named groups per stage.
- **Simulator**: semi-implicit Euler at `dt = 0.25 s`. Mass drops as fuel
  burns (`mdot = thrust / (isp * g0)`); drag is `½ρv²CdA` with an
  exponential atmosphere; gravity weakens with altitude. Events fire when
  the timeline crosses them; max-Q is tracked every step.

## Try changing

- Give the boosters more burn time, or add a third stage — watch the
  apogee move.
- Steepen the pitch program in `pitch_from_vertical()` and see max-Q climb.
- Design your own rocket with `Rocket` / `Stage` / `Booster` and fly it.

Built by [Mohammad Ali](https://github.com/AliRender13).
