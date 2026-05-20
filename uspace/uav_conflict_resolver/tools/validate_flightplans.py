"""Simple validator for kinematic feasibility of generated FlightPlans (viz_08).

Checks per-segment average linear speed against `max_var_lin_vel`, checks
angular velocity between consecutive velocity vectors against
`max_var_ang_vel`, and ensures `heading` is present on waypoints.
"""
import numpy as np

from uspace.uav_conflict_resolver.visualizers.viz_08_full_cascade_scenario import generate_straight_fleet
from uspace.uav_conflict_resolver.core.config import UAV_MAX_SPEED


def validate(n_uavs: int = 5):
    SPACE = generate_straight_fleet(n_uavs)
    issues = []

    for fp in SPACE:
        fid = getattr(fp, 'id', '<no-id>')
        for wp in fp.waypoints:
            if getattr(wp, 'heading', None) is None:
                issues.append(f"UAV_{fid}: waypoint '{wp.label}' missing heading")

        for a, b in zip(fp.waypoints, fp.waypoints[1:]):
            dt = b.t - a.t
            if dt <= 0:
                issues.append(f"UAV_{fid}: non-positive dt between '{a.label}' and '{b.label}': {dt}")
                continue

            pa = np.array(a.pos)
            pb = np.array(b.pos)
            dist = np.linalg.norm(pb - pa)
            avg_speed = dist / dt

            # Check absolute feasibility against UAV_MAX_SPEED
            if avg_speed > UAV_MAX_SPEED + 1e-6:
                issues.append(f"UAV_{fid}: segment {a.label}->{b.label} avg speed {avg_speed:.2f} > UAV_MAX_SPEED {UAV_MAX_SPEED}")

            # Check variation in linear speed between consecutive WPs (max_var_lin_vel)
            v1 = np.array(a.vel) if getattr(a, 'vel', None) is not None else (pb - pa) / dt
            v2 = np.array(b.vel) if getattr(b, 'vel', None) is not None else (pb - pa) / dt

            n1 = np.linalg.norm(v1)
            n2 = np.linalg.norm(v2)
            max_lin_var = getattr(fp, 'max_var_lin_vel', None)
            if max_lin_var is not None:
                if abs(n2 - n1) > max_lin_var + 1e-6:
                    issues.append(f"UAV_{fid}: linear speed change |{n2:.2f}-{n1:.2f}| = {abs(n2-n1):.2f} > max_var_lin_vel {max_lin_var}")

            if n1 > 0 and n2 > 0:
                cos = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
                angle = np.arccos(cos)
                ang_vel = angle / dt
                max_ang = getattr(fp, 'max_var_ang_vel', None)
                if max_ang is not None and ang_vel > max_ang + 1e-6:
                    issues.append(f"UAV_{fid}: angular vel between {a.label}->{b.label} {ang_vel:.2f} > max {max_ang}")

    # Print summary
    print("\nFlightPlan Kinematic Validation Report:\n")
    if not issues:
        print("  OK: no issues found. FlightPlans appear kinematically feasible.")
    else:
        print(f"  Found {len(issues)} issue(s):")
        for it in issues:
            print("   -", it)

    return issues


if __name__ == '__main__':
    validate(5)
