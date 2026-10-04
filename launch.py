"""launch.py - launch trajectory simulator for procedural-rocket.

Flies a multi-stage Rocket (from stages.py) through a 2-DOF simulation:
thrust, gravity, atmospheric drag, staging events, booster separation,
max-Q detection and apogee. Prints a mini mission-control readout with
ASCII altitude/velocity plots.

Pure Python, standard library only.  Run:  python3 launch.py
"""
import math
from stages import G0, demo_rocket

R_EARTH = 6371000.0   # m
RHO0 = 1.225          # sea-level air density, kg/m^3
H_SCALE = 8500.0      # atmosphere scale height, m
CD = 0.32             # drag coefficient (slender rocket)


def atmosphere_rho(alt_m):
    return RHO0 * math.exp(-max(0.0, alt_m) / H_SCALE)


def gravity(alt_m):
    return G0 * (R_EARTH / (R_EARTH + max(0.0, alt_m))) ** 2


def pitch_from_vertical(t):
    """Pitch program: degrees away from vertical (gravity turn)."""
    if t < 10.0:
        return 0.0
    if t < 200.0:
        return 55.0 * (t - 10.0) / 190.0
    return 55.0


def simulate(rocket, dt=0.25, t_max=1500.0):
    """Fly the rocket. Returns dict with time series, events, max_q, apogee."""
    n = len(rocket.stages)
    t_ign = [0.0] * n
    for k in range(1, n):
        t_ign[k] = t_ign[k - 1] + rocket.stages[k - 1].burn_s
    t_bsep = max((b.burn_s for b in rocket.boosters), default=0.0)
    t_burnout = [t_ign[k] + rocket.stages[k].burn_s for k in range(n)]

    fuel_b = [b.fuel_kg for b in rocket.boosters]
    fuel_s = [s.fuel_kg for s in rocket.stages]

    def mass_at(t):
        m = rocket.payload_kg
        for k, s in enumerate(rocket.stages):
            if t >= t_ign[k + 1] if k + 1 < n else False:
                continue  # jettisoned
            m += s.dry_kg + fuel_s[k]
        if t < t_bsep:
            for i, b in enumerate(rocket.boosters):
                m += b.dry_kg + fuel_b[i]
        return m

    def thrust_at(t):
        T = 0.0
        active = []
        for i, b in enumerate(rocket.boosters):
            if t < b.burn_s and fuel_b[i] > 0:
                T += b.thrust_n
                active.append(("b", i))
        for k, s in enumerate(rocket.stages):
            if t_ign[k] <= t < t_burnout[k] and fuel_s[k] > 0:
                T += s.thrust_n
                active.append(("s", k))
        return T, active

    t, x, y = 0.0, 0.0, 0.0
    vx, vy = 0.0, 0.0
    area = math.pi * rocket.stages[0].radius_m ** 2

    series = {"t": [], "alt": [], "down": [], "vel": [], "twr": [], "mass": [], "q": []}
    events = [dict(t=0.0, name="LIFTOFF", alt=0.0, vel=0.0)]
    fired = set()
    max_q, max_q_t, max_q_alt, max_q_vel = 0.0, 0.0, 0.0, 0.0
    apogee, apogee_t, apogee_down = None, None, None
    prev_vy = 0.0

    def note(name):
        if name not in fired:
            fired.add(name)
            events.append(dict(t=t, name=name, alt=y,
                               vel=math.hypot(vx, vy)))

    while t <= t_max:
        m = mass_at(t)
        T, active = thrust_at(t)
        for kind, i in active:
            spec = rocket.boosters[i] if kind == "b" else rocket.stages[i]
            fuel = fuel_b if kind == "b" else fuel_s
            burn = min(fuel[i], spec.mdot_kg_s * dt)
            fuel[i] -= burn
        v = math.hypot(vx, vy)
        rho = atmosphere_rho(y)
        q = 0.5 * rho * v * v
        if q > max_q:
            max_q, max_q_t, max_q_alt = q, t, y
            max_q_vel = math.hypot(vx, vy)
        g = gravity(y)
        p = math.radians(pitch_from_vertical(t))
        tx, ty = math.sin(p), math.cos(p)
        D = q * CD * area
        if v > 1e-9:
            ax = (T * tx - D * vx / v) / m
            ay = (T * ty - D * vy / v) / m - g
        else:
            ax, ay = T * tx / m, T * ty / m - g
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt
        t += dt

        twr = T / (m * g) if m > 0 else 0.0
        series["t"].append(t)
        series["alt"].append(max(0.0, y))
        series["down"].append(x)
        series["vel"].append(math.hypot(vx, vy))
        series["twr"].append(twr)
        series["mass"].append(m)
        series["q"].append(q)

        if t_bsep and t >= t_bsep:
            note("BOOSTER SEP")
        for k in range(n):
            if t >= t_burnout[k]:
                note(f"MECO-{k + 1}")
                if k + 1 < n:
                    note(f"STAGE {k + 1} SEP")
                    note(f"STAGE-{k + 2} IGNITION")
        if n and t >= t_burnout[-1]:
            note("SECO")
        if apogee is None and prev_vy > 0 and vy <= 0 and y > 1000:
            apogee, apogee_t, apogee_down = y, t, x
            events.append(dict(t=t, name="APOGEE", alt=y,
                               vel=math.hypot(vx, vy)))
        prev_vy = vy
        if apogee is not None and (y <= 0 or t > apogee_t + 60):
            break

    events.append(dict(t=max_q_t, name="MAX-Q", alt=max_q_alt, vel=max_q_vel))
    events.sort(key=lambda e: e["t"])
    return {
        "rocket": rocket, "series": series, "events": events,
        "t_ign": t_ign, "t_burnout": t_burnout, "t_bsep": t_bsep,
        "max_q": max_q, "max_q_t": max_q_t, "max_q_alt": max_q_alt,
        "apogee": apogee, "apogee_t": apogee_t, "apogee_down": apogee_down,
    }


def ascii_plot(ts, ys, events, width=68, height=15, title="",
               y_fmt=lambda v: f"{v:7.1f}"):
    """Draw a labelled ASCII time plot; events marked with letters."""
    lo_t, hi_t = ts[0], ts[-1]
    hi_y = max(ys) * 1.05 or 1.0
    grid = [[" "] * width for _ in range(height)]

    def px(t):
        return max(0, min(width - 1, int((t - lo_t) / (hi_t - lo_t) * (width - 1))))

    def py(v):
        return height - 1 - min(height - 1, int(v / hi_y * (height - 1)))

    step = max(1, len(ts) // (width * 2))
    for i in range(0, len(ts), step):
        grid[py(ys[i])][px(ts[i])] = "*"

    marks = {}
    for n, e in enumerate(events):
        ch = chr(ord("A") + n % 26)
        if e["t"] <= hi_t:
            idx = min(range(len(ts)), key=lambda i: abs(ts[i] - e["t"]))
            grid[py(ys[idx])][px(ts[idx])] = ch
            marks[ch] = e["name"]

    lines = [f"  {title}"]
    for r in range(height):
        v = hi_y * (height - 1 - r) / (height - 1)
        lines.append(f"{y_fmt(v)} |{''.join(grid[r])}|")
    lines.append(" " * 9 + "+" + "-" * width + "+")
    tlab = f"{lo_t:6.0f}s{' ' * (width - 16)}{hi_t:6.0f}s"
    lines.append(" " * 9 + tlab)
    if marks:
        leg = "   ".join(f"{c}={name}" for c, name in marks.items())
        lines.append("  events: " + leg)
    return "\n".join(lines)


def stage_summary(res):
    """Per-stage numbers: prop used, delta-v, max TWR, burnout state."""
    r = res["rocket"]
    s, ev = res["series"], res["events"]
    n = len(r.stages)
    out = []
    for k, st in enumerate(r.stages):
        t0, t1 = res["t_ign"][k], res["t_burnout"][k]
        i0 = min(range(len(s["t"])), key=lambda i: abs(s["t"][i] - t0))
        i1 = min(range(len(s["t"])), key=lambda i: abs(s["t"][i] - t1))
        m0, m1 = s["mass"][i0], s["mass"][i1]
        dv = st.isp_s * G0 * math.log(m0 / m1) if m1 > 0 else 0.0
        twrs = s["twr"][i0:i1 + 1]
        out.append(dict(name=st.name, t0=t0, t1=t1,
                        prop_kg=st.fuel_kg, dv=dv,
                        max_twr=max(twrs) if twrs else 0.0,
                        alt1=s["alt"][i1] / 1000, vel1=s["vel"][i1]))
    if r.boosters:
        b = r.boosters[0]
        i1 = min(range(len(s["t"])), key=lambda i: abs(s["t"][i] - res["t_bsep"]))
        m0 = s["mass"][0]
        dv = b.isp_s * G0 * math.log(m0 / s["mass"][i1]) if s["mass"][i1] > 0 else 0.0
        out.insert(0, dict(name=f"{len(r.boosters)}x {b.name[:-2]}",
                           t0=0.0, t1=res["t_bsep"],
                           prop_kg=sum(x.fuel_kg for x in r.boosters), dv=dv,
                           max_twr=max(s["twr"][:i1 + 1]),
                           alt1=s["alt"][i1] / 1000, vel1=s["vel"][i1]))
    return out


def print_report(res):
    r = res["rocket"]
    s = res["series"]
    W = 64
    print("=" * W)
    print(f"  {r.name}  *  MISSION CONTROL READOUT".center(W - 2))
    print("=" * W)
    print(f"  liftoff mass    {s['mass'][0]:>11,.0f} kg")
    print(f"  liftoff TWR     {s['twr'][0]:>11.2f}")
    print(f"  max-Q           {res['max_q'] / 1000:>8.1f} kPa"
          f"   @ T+{res['max_q_t']:5.0f}s, alt {res['max_q_alt'] / 1000:5.1f} km")
    print()
    print("  EVENT LOG")
    for e in res["events"]:
        print(f"    T+{e['t']:6.1f}s  {e['name']:<18}"
              f" alt {e['alt'] / 1000:7.1f} km   vel {e['vel']:6.0f} m/s")
    print()
    print("  STAGE SUMMARY")
    print(f"    {'stage':<14}{'burn':>10}{'prop':>10}"
          f"{'delta-v':>9}{'max TWR':>9}{'alt':>9}")
    for st in stage_summary(res):
        print(f"    {st['name']:<14}T+{st['t0']:4.0f}-{st['t1']:4.0f}s"
              f"{st['prop_kg'] / 1000:>8.0f}t{st['dv']:>8.0f}m/s"
              f"{st['max_twr']:>9.2f}{st['alt1']:>8.1f}km")
    print()
    if res["apogee"]:
        print(f"  APOGEE  {res['apogee'] / 1000:.1f} km"
              f"   @ T+{res['apogee_t']:.0f}s,"
              f" downrange {res['apogee_down'] / 1000:.0f} km")
    print("=" * W)
    print()
    print(ascii_plot(s["t"], [a / 1000 for a in s["alt"]], res["events"],
                     title="ALTITUDE vs TIME  (km)",
                     y_fmt=lambda v: f"{v:7.1f}"))
    print()
    print(ascii_plot(s["t"], s["vel"], res["events"],
                     title="VELOCITY vs TIME  (m/s)",
                     y_fmt=lambda v: f"{v:7.0f}"))


def main():
    rocket = demo_rocket()
    print(f"simulating {rocket.name} ...")
    res = simulate(rocket)
    print()
    print_report(res)


if __name__ == "__main__":
    main()
