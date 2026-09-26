# procedural-rocket

Generates a **low-poly 3D rocket model** in **pure Python** — no dependencies.

## Run it

```bash
python3 rocket.py
```

It writes `rocket.obj`: a rocket with a cylinder body, cone nose and three
fins. Open it in **Blender**, Windows 3D Viewer, or an online viewer like
3dviewer.net.

## How it works

- Rings of vertices are placed with `cos`/`sin`, then stitched into
  triangles: cylinder wall, bottom cap fan, nose-cone fan.
- Fins are simple triangular quads angled 120° apart.
- Everything is written by hand in the Wavefront OBJ format (`v` for
  vertices, `f` for faces) — no 3D library needed.

## Try changing

- `SEGMENTS` for a chunkier or smoother rocket.
- Add a window (a small flattened box) or an engine bell (an open cone).

Built by [Mohammad Ali](https://github.com/AliRender13).
