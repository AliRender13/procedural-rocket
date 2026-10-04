"""
stages.py - multi-stage rocket builder for procedural-rocket.

Stack stages and strap on side boosters, each with its own propulsion
specs (thrust, burn time, dry/fuel mass, ISP), then export the whole
launch vehicle as one Wavefront OBJ file.

Pure Python, standard library only.

    from stages import Stage, Booster, Rocket, demo_rocket

    r = demo_rocket()          # Aurora-9: 2 stages + 2 boosters
    r.write_obj("aurora9.obj") # open it in Blender / 3dviewer.net
"""
import math

G0 = 9.80665  # standard gravity, m/s^2


class Stage:
    """One rocket stage: a cylinder body with an engine bell."""

    def __init__(self, name, thrust_kn, burn_s, dry_kg, fuel_kg,
                 radius_m, height_m, isp_s=282.0):
        self.name = name
        self.thrust_kn = float(thrust_kn)
        self.burn_s = float(burn_s)
        self.dry_kg = float(dry_kg)
        self.fuel_kg = float(fuel_kg)
        self.radius_m = float(radius_m)
        self.height_m = float(height_m)
        self.isp_s = float(isp_s)

    @property
    def thrust_n(self):
        return self.thrust_kn * 1000.0

    @property
    def wet_kg(self):
        return self.dry_kg + self.fuel_kg

    @property
    def mdot_kg_s(self):
        """Propellant mass flow, kg/s."""
        return self.thrust_n / (self.isp_s * G0)

    def __repr__(self):
        return (f"Stage({self.name!r}, {self.thrust_kn:.0f} kN, "
                f"{self.burn_s:.0f}s, {self.wet_kg:,.0f} kg)")


class Booster(Stage):
    """A side booster strapped to the core, offset sideways in metres."""

    def __init__(self, name, offset_m, **kw):
        super().__init__(name, **kw)
        self.offset_m = float(offset_m)


class _Mesh:
    """Tiny OBJ mesh accumulator: vertices, triangular faces, groups."""

    def __init__(self):
        self.verts = []    # [(x, y, z), ...]
        self.faces = []    # [(i, j, k), ...] 1-based indices
        self._gstarts = []  # [(name, face_start_0based), ...]

    def v(self, x, y, z):
        self.verts.append((x, y, z))
        return len(self.verts)  # 1-based id

    def group(self, name):
        self._gstarts.append((name, len(self.faces)))

    def groups(self):
        out = []
        for i, (name, s) in enumerate(self._gstarts):
            e = self._gstarts[i + 1][1] if i + 1 < len(self._gstarts) else len(self.faces)
            out.append((name, s, e))
        return out

    def ring(self, y, r, cx=0.0, cz=0.0, n=16):
        ids = []
        for i in range(n):
            a = 2 * math.pi * i / n
            ids.append(self.v(cx + r * math.cos(a), y, cz + r * math.sin(a)))
        return ids

    def tube(self, y0, y1, r, cx=0.0, cz=0.0, n=16, cap0=False, cap1=False):
        lo = self.ring(y0, r, cx, cz, n)
        hi = self.ring(y1, r, cx, cz, n)
        for i in range(n):
            j = (i + 1) % n
            self.faces.append((lo[i], lo[j], hi[j]))
            self.faces.append((lo[i], hi[j], hi[i]))
        if cap0:
            c = self.v(cx, y0, cz)
            for i in range(n):
                j = (i + 1) % n
                self.faces.append((c, lo[j], lo[i]))
        if cap1:
            c = self.v(cx, y1, cz)
            for i in range(n):
                j = (i + 1) % n
                self.faces.append((c, hi[i], hi[j]))

    def cone(self, y0, y1, r, cx=0.0, cz=0.0, n=16):
        base = self.ring(y0, r, cx, cz, n)
        apex = self.v(cx, y1, cz)
        for i in range(n):
            j = (i + 1) % n
            self.faces.append((apex, base[i], base[j]))

    def bell(self, y, r, n=16, cx=0.0, cz=0.0):
        """Engine bell: a flared open nozzle hanging below y."""
        top = self.ring(y, r * 0.55, cx, cz, n)
        bot = self.ring(y - r * 0.9, r, cx, cz, n)
        for i in range(n):
            j = (i + 1) % n
            self.faces.append((top[i], top[j], bot[j]))
            self.faces.append((top[i], bot[j], bot[i]))

    def fin(self, y0, y1, r_body, r_tip, angle, cx=0.0, cz=0.0):
        dx, dz = math.cos(angle), math.sin(angle)
        p1 = self.v(cx + dx * r_body, y0, cz + dz * r_body)
        p2 = self.v(cx + dx * r_body, y1, cz + dz * r_body)
        p3 = self.v(cx + dx * r_tip, y0, cz + dz * r_tip)
        self.faces.append((p1, p2, p3))


class Rocket:
    """A full launch vehicle: stacked stages, side boosters, payload."""

    def __init__(self, name):
        self.name = name
        self.stages = []    # bottom -> top
        self.boosters = []  # side boosters
        self.payload_kg = 0.0

    def add_stage(self, stage):
        self.stages.append(stage)
        return self

    def add_booster(self, booster):
        self.boosters.append(booster)
        return self

    def set_payload(self, kg):
        self.payload_kg = float(kg)
        return self

    def liftoff_mass(self):
        return (self.payload_kg
                + sum(s.wet_kg for s in self.stages)
                + sum(b.wet_kg for b in self.boosters))

    def liftoff_thrust_n(self):
        return (sum(s.thrust_n for s in self.stages[:1])
                + sum(b.thrust_n for b in self.boosters))

    def liftoff_twr(self):
        return self.liftoff_thrust_n() / (self.liftoff_mass() * G0)

    def stack_height(self):
        h = sum(s.height_m for s in self.stages)
        h += 0.8 * max(0, len(self.stages) - 1)  # interstages
        h += self.stages[-1].radius_m * 1.6      # nose cone
        return h

    def spec_table(self):
        rows = []
        for b in self.boosters:
            rows.append(("booster", b.name, b.thrust_kn, b.burn_s,
                         b.dry_kg, b.fuel_kg, b.isp_s))
        for i, s in enumerate(self.stages, 1):
            rows.append((f"stage {i}", s.name, s.thrust_kn, s.burn_s,
                         s.dry_kg, s.fuel_kg, s.isp_s))
        return rows

    def build_mesh(self, segments=16):
        m = _Mesh()
        y = 0.0
        for i, st in enumerate(self.stages):
            m.group(f"stage{i + 1}_{st.name}")
            m.bell(y + 0.01, st.radius_m * 0.8, segments)
            m.tube(y, y + st.height_m, st.radius_m, n=segments,
                   cap0=(i == 0))
            y += st.height_m
            if i < len(self.stages) - 1:  # interstage band
                m.tube(y, y + 0.8, st.radius_m * 1.02, n=segments)
                y += 0.8
        m.group("nose_cone")
        top_r = self.stages[-1].radius_m
        m.cone(y, y + top_r * 1.6, top_r, n=segments)
        for b in self.boosters:
            m.group(f"booster_{b.name}")
            bx = b.offset_m
            m.bell(0.01, b.radius_m * 0.8, segments, cx=bx)
            m.tube(0.0, b.height_m, b.radius_m, cx=bx, n=segments, cap0=True)
            m.cone(b.height_m, b.height_m + b.radius_m * 1.6,
                   b.radius_m, cx=bx, n=segments)
            for k in range(3):
                m.fin(0.2, 2.4, b.radius_m, b.radius_m + 1.0,
                      2 * math.pi * k / 3, cx=bx)
        m.group("core_fins")
        r0 = self.stages[0].radius_m
        for k in range(4):
            m.fin(0.3, 3.2, r0, r0 + 1.7, math.pi / 4 + k * math.pi / 2)
        return m

    def write_obj(self, path, segments=16):
        m = self.build_mesh(segments)
        with open(path, "w") as f:
            f.write(f"# {self.name} - multi-stage launch vehicle "
                    f"generated by procedural-rocket\n")
            f.write(f"o {self.name.replace(' ', '_')}\n")
            for x, y, z in m.verts:
                f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
            for gname, s, e in m.groups():
                f.write(f"g {gname}\n")
                for i, j, k in m.faces[s:e]:
                    f.write(f"f {i} {j} {k}\n")
        return len(m.verts), len(m.faces)


def demo_rocket():
    """Aurora-9: 2 stages + 2 side boosters, ~837 t on the pad."""
    r = Rocket("Aurora-9")
    for side, name in ((3.4, "SRB-1"), (-3.4, "SRB-2")):
        r.add_booster(Booster(name, offset_m=side,
                              thrust_kn=2100, burn_s=62,
                              dry_kg=9000, fuel_kg=125000,
                              radius_m=1.25, height_m=24, isp_s=273))
    r.add_stage(Stage("Stage-1", thrust_kn=7800, burn_s=152,
                      dry_kg=23000, fuel_kg=410000,
                      radius_m=1.9, height_m=42, isp_s=282))
    r.add_stage(Stage("Stage-2", thrust_kn=980, burn_s=320,
                      dry_kg=4500, fuel_kg=115000,
                      radius_m=1.9, height_m=15, isp_s=348))
    r.set_payload(16000)
    return r


def main():
    r = demo_rocket()
    print(f"{r.name}: {len(r.stages)} stages + {len(r.boosters)} boosters")
    print(f"  liftoff mass   {r.liftoff_mass():>12,.0f} kg")
    print(f"  liftoff thrust {r.liftoff_thrust_n() / 1000:>12,.0f} kN")
    print(f"  liftoff TWR    {r.liftoff_twr():>12.2f}")
    print(f"  stack height   {r.stack_height():>12.1f} m")
    print()
    print(f"  {'part':<10}{'name':<10}{'thrust':>8}{'burn':>7}"
          f"{'dry':>10}{'fuel':>10}{'isp':>7}")
    for kind, name, th, bu, dry, fuel, isp in r.spec_table():
        print(f"  {kind:<10}{name:<10}{th:>7.0f}kN{bu:>6.0f}s"
              f"{dry:>9,.0f}k{fuel:>9,.0f}k{isp:>6.0f}s")
    nv, nf = r.write_obj("aurora9.obj")
    print(f"\naurora9.obj written: {nv} vertices, {nf} faces")


if __name__ == "__main__":
    main()
