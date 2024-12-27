import math
import matplotlib.pyplot as plt

AIR_DENSITY = 1.225  # kg/m^3 (approx. at sea level)
G = 9.81             # gravitational acceleration (m/s^2)

# 1) Weather friction
WEATHER_CONDITIONS = {
    "1": ("Dry asphalt", 0.85),
    "2": ("Wet asphalt", 0.55),
    "3": ("Snow", 0.20),
    "4": ("Ice", 0.10)
}

# 2) Tyre quality multipliers
TYRE_QUALITY = {
    "1": ("Good (new / healthy tread)", 1.0),
    "2": ("Decent (average)", 0.8),
    "3": ("Poor (low tread)", 0.5)
}

# 3) Car database: (mass_kg, Cd, frontal_area_m2)
#    Approximations only; actual values vary by year/model/trim.
CAR_DATABASE = {
    "Toyota HiLux":       (2100, 0.40, 2.5),
    "Toyota Corolla":     (1300, 0.29, 2.2),
    "Toyota RAV4":        (1600, 0.33, 2.7),
    "Mazda CX-5":         (1600, 0.33, 2.7),
    "Hyundai i30":        (1300, 0.30, 2.2),
    "Toyota Camry":       (1500, 0.28, 2.2),
    "Mazda3":             (1300, 0.28, 2.2),
    "Mitsubishi Triton":  (2000, 0.42, 2.7),
    "Nissan X-Trail":     (1500, 0.35, 2.7),
    "Subaru Forester":    (1500, 0.35, 2.7)
}

# 4) Slope menu => store slope% (negative=downhill, positive=uphill)
SLOPE_MENU = {
    "1": ("Flat or near-flat", 0.0),
    "2": ("Slight uphill", 2.0),
    "3": ("Moderate uphill", 5.0),
    "4": ("Slight downhill", -2.0),
    "5": ("Moderate downhill", -5.0),
    "6": ("Steep downhill", -8.0)
}

def convert_kmh_to_ms(speed_kmh: float) -> float:
    """Convert km/h to m/s."""
    return speed_kmh * 1000.0 / 3600.0

def convert_ms_to_kmh(speed_ms: float) -> float:
    """Convert m/s back to km/h."""
    return speed_ms * 3.6

def calc_final_friction(weather_mu, tyre_factor, abs_active):
    """
    Combine weather friction + tyre factor + optional ABS bump.
    E.g., final_mu = (weather_mu * tyre_factor) + (0.05 if ABS else 0).
    """
    mu = weather_mu * tyre_factor
    if abs_active:
        mu += 0.05
    return max(mu, 0.01)  # avoid zero or negative friction

def stopping_distance_simple_friction(speed_kmh, mu, slope_percent, reaction_time):
    """
    Simpler friction-based formula ignoring drag:
       a_eff = - mu*g*cos(alpha) + slope_acc
    where slope_acc = +/- g*sin(alpha) depending on uphill/downhill.
    Returns (total_dist, braking_dist, reaction_dist).
    """
    speed_ms = convert_kmh_to_ms(speed_kmh)
    alpha = math.atan(abs(slope_percent) / 100.0)
    is_uphill = (slope_percent >= 0)

    # Reaction distance
    reaction_dist = speed_ms * reaction_time

    # Slope contribution
    slope_acc = (-G * math.sin(alpha)) if is_uphill else (G * math.sin(alpha))

    # Net acceleration
    a_eff = - mu * G * math.cos(alpha) + slope_acc
    if a_eff <= 0:
        return float('inf'), float('inf'), reaction_dist  # can't stop under these conditions

    # Braking distance via v^2 = 2*a_eff*d => d = v^2 / (2*a_eff)
    braking_dist = (speed_ms ** 2) / (2 * a_eff)
    total_dist = reaction_dist + braking_dist
    return total_dist, braking_dist, reaction_dist

def stopping_distance_numeric_drag(
    speed_kmh, mu, mass_kg, cd, frontal_area, slope_percent, reaction_time, dt=0.01
):
    """
    Numerical approach with drag + friction + slope fix:
       a = - mu*g*cos(alpha) + slope_term - drag_term
    Returns (dist_total, dist_reaction, dist_braking, total_time, final_v).
    """
    speed_ms = convert_kmh_to_ms(speed_kmh)

    # Reaction distance => no deceleration
    reaction_dist = speed_ms * reaction_time

    alpha = math.atan(abs(slope_percent) / 100.0)
    is_uphill = (slope_percent >= 0)

    v = speed_ms
    x = 0.0
    t = 0.0
    max_time = 300.0  # 5 min cap

    while t < max_time:
        # Drag
        drag_acc = (0.5 * AIR_DENSITY * cd * frontal_area * (v ** 2)) / mass_kg
        # Slope
        slope_acc = (-G * math.sin(alpha)) if is_uphill else (G * math.sin(alpha))
        # Net acceleration
        a = - mu * G * math.cos(alpha) + slope_acc - drag_acc

        v_new = v + a * dt
        if v_new < 0:
            # zero within dt
            dist_this_step = 0.5 * (v + 0.0) * dt
            x += dist_this_step
            v_new = 0
            break

        # distance covered in this step
        dist_this_step = 0.5 * (v + v_new) * dt
        x += dist_this_step

        v = v_new
        t += dt

        if v <= 0.01:  # near zero
            v = 0
            break

    braking_dist = x
    total_dist = reaction_dist + braking_dist
    total_time = reaction_time + t
    return total_dist, reaction_dist, braking_dist, total_time, v

def get_distance_speed_profile(speed_kmh, mu, mass_kg, cd, frontal_area,
                               slope_percent, reaction_time, dt=0.05):
    """
    Returns parallel lists of (distance array, speed array in km/h).
    We track distance cumulatively and convert speed to km/h.
    """
    speed_ms = convert_kmh_to_ms(speed_kmh)

    distance_vals = []
    speed_vals_kmh = []
    alpha = math.atan(abs(slope_percent) / 100.0)
    is_uphill = (slope_percent >= 0)

    # Reaction distance => no deceleration
    reaction_dist = speed_ms * reaction_time

    # We'll do the integration in steps: first for the reaction phase
    # (constant speed, no deceleration).
    x = 0.0
    v = speed_ms
    t = 0.0
    max_time = 300.0

    # (1) Reaction phase
    while t <= reaction_time:
        distance_vals.append(x)
        speed_vals_kmh.append(convert_ms_to_kmh(v))  # convert to km/h
        t += dt
        x += v * dt  # traveling at constant speed

    # (2) Braking phase
    while t <= (reaction_time + max_time):
        # drag
        drag_acc = 0.5 * AIR_DENSITY * cd * frontal_area * (v ** 2) / mass_kg
        # slope
        slope_acc = (-G * math.sin(alpha)) if is_uphill else (G * math.sin(alpha))
        # net
        a = - mu * G * math.cos(alpha) + slope_acc - drag_acc

        v_new = v + a * dt
        if v_new < 0:
            v_new = 0

        distance_vals.append(x)
        speed_vals_kmh.append(convert_ms_to_kmh(v_new))

        # distance for this small interval
        avg_speed = 0.5 * (v + v_new)
        x += avg_speed * dt

        v = v_new
        t += dt
        if v <= 0.01:
            break

    return distance_vals, speed_vals_kmh

def main():
    print("=== Stopping Distance Calculator (Fixed Slope Sign) ===")

    # 1) Car selection
    print("\nSelect your car from the list below or type 'O' for manual mass entry:")
    car_keys = list(CAR_DATABASE.keys())
    for i, ck in enumerate(car_keys, start=1):
        print(f"{i}. {ck}")
    print("O. Other (manual entry)")

    user_car_choice = input("> ").strip().upper()
    known_car = False
    if user_car_choice == "O":
        # Manual mass
        while True:
            try:
                user_mass = float(input("Enter your car's mass in kg: "))
                if user_mass <= 0:
                    raise ValueError
                car_mass = user_mass
                c_d = 0.3
                f_area = 2.2
                break
            except ValueError:
                print("Invalid mass, please try again.")
        print("\n** NOTE ** Using simpler friction model (no drag). Results less accurate.\n")
    else:
        try:
            idx = int(user_car_choice)
            if idx < 1 or idx > len(car_keys):
                raise ValueError
            chosen_car = car_keys[idx - 1]
            (car_mass, c_d, f_area) = CAR_DATABASE[chosen_car]
            known_car = True
            print(f"Selected car: {chosen_car}")
            print(f"  - mass={car_mass} kg")
        except:
            print("Invalid selection. Defaulting to Toyota Corolla.")
            chosen_car = "Toyota Corolla"
            (car_mass, c_d, f_area) = CAR_DATABASE[chosen_car]
            known_car = True

    # 2) Weather
    print("\nSelect road/weather condition:")
    for k, (desc, muval) in WEATHER_CONDITIONS.items():
        print(f"{k}. {desc}")
    w_choice = input("> ").strip()
    if w_choice in WEATHER_CONDITIONS:
        (weather_desc, base_mu) = WEATHER_CONDITIONS[w_choice]
    else:
        (weather_desc, base_mu) = WEATHER_CONDITIONS["1"]

    # 3) Tyre quality
    print("\nSelect tyre condition:")
    for tk, (tdesc, tfact) in TYRE_QUALITY.items():
        print(f"{tk}. {tdesc}")
    tyre_choice = input("> ").strip()
    if tyre_choice in TYRE_QUALITY:
        (tyre_desc, tyre_factor) = TYRE_QUALITY[tyre_choice]
    else:
        (tyre_desc, tyre_factor) = TYRE_QUALITY["2"]

    # 4) ABS
    abs_choice = input("\nDoes your car have ABS? (y/n): ").strip().lower()
    abs_active = (abs_choice == "y")

    # 5) Reaction time
    print("\nAre you tired or well-rested?")
    print("1. Not tired")
    print("2. Tired")
    rt_choice = input("> ").strip()
    reaction_time = 2.0 if rt_choice == "2" else 1.0

    # 6) Slope
    print("\nSelect road slope:")
    for sk, (sd, val) in SLOPE_MENU.items():
        print(f"{sk}. {sd}")
    slope_in = input("> ").strip()
    if slope_in in SLOPE_MENU:
        slope_desc, slope_val = SLOPE_MENU[slope_in]
    else:
        slope_desc, slope_val = SLOPE_MENU["1"]  # default flat

    # 7) Speed
    try:
        speed_kmh = float(input("\nEnter speed in km/h: "))
    except ValueError:
        speed_kmh = 60.0

    # Combine friction
    mu_final = calc_final_friction(base_mu, tyre_factor, abs_active)

    # Show summary
    print("\n=== INPUT SUMMARY ===")
    if known_car:
        print(f"Car: {chosen_car} (mass={car_mass} kg)")
    else:
        print(f"Manual mass: {car_mass} kg (no drag calc).")
    print(f"Weather: {weather_desc}, base mu={base_mu:.2f}")
    print(f"Tyres: {tyre_desc}, ABS={abs_active} => final friction={mu_final:.2f}")
    print(f"Slope: {slope_desc} ({slope_val:+.1f}%)")
    print(f"Reaction time: {reaction_time:.2f} s")
    print(f"Speed: {speed_kmh:.2f} km/h")

    # 8) Compute distances
    if known_car:
        # Use numeric approach with drag
        (dist_total, dist_react, dist_brake, t_total, v_end) = stopping_distance_numeric_drag(
            speed_kmh, mu_final, car_mass, c_d, f_area, slope_val, reaction_time
        )
        if dist_total == float('inf'):
            print("\nCar cannot stop (net deceleration <= 0).")
            return
        print("\n=== RESULTS (NUMERIC + DRAG) ===")
        print(f"Reaction distance: {dist_react:.2f} m")
        print(f"Braking distance:  {dist_brake:.2f} m")
        print(f"TOTAL distance:    {dist_total:.2f} m")
        print(f"Total time:        {t_total:.2f} s")
        print(f"Final velocity:    {v_end:.2f} m/s (should be ~0 if fully stopped)")

        # ------------------------------------------------------------------
        # SINGLE FIGURE: 2×2 subplots, each shows distance (x-axis, m)
        # vs. speed in km/h (y-axis), using dashed lines.
        #
        # We replace "baseline" with the actual user-chosen speed label,
        # e.g. "60 km/h" if offset=0.
        # ------------------------------------------------------------------
        speed_offsets = [-20, -10, 0, 10, 20]  # km/h
        weather_variants = [
            ("Dry", 0.85),
            ("Wet", 0.55),
        ]
        reaction_variants = [
            ("Alert (1s)", 1.0),
            ("Tired (2s)", 2.0),
        ]

        fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(12, 8), sharex=False, sharey=False)

        for col_idx, (w_label, w_mu) in enumerate(weather_variants):
            for row_idx, (rt_label, rt_val) in enumerate(reaction_variants):
                ax = axes[row_idx, col_idx]
                ax.set_title(f"{w_label} | {rt_label}", fontsize=11)

                for offset in speed_offsets:
                    scenario_speed = speed_kmh + offset
                    if scenario_speed < 0:
                        scenario_speed = 0  # no negative speeds

                    # Adjust friction for scenario
                    scenario_mu = calc_final_friction(w_mu, tyre_factor, abs_active)

                    # distance vs. speed-in-kmh profile
                    dist_arr, spd_kmh_arr = get_distance_speed_profile(
                        speed_kmh=scenario_speed,
                        mu=scenario_mu,
                        mass_kg=car_mass,
                        cd=c_d,
                        frontal_area=f_area,
                        slope_percent=slope_val,
                        reaction_time=rt_val,
                        dt=0.05
                    )

                    # If offset=0, show the actual user-chosen speed (e.g. "60 km/h")
                    if offset == 0:
                        offset_str = f"{speed_kmh:.0f} km/h"
                    elif offset > 0:
                        offset_str = f"+{offset} km/h"
                    else:
                        offset_str = f"{offset} km/h"

                    # Plot dashed line: X=distance (m), Y=speed (km/h)
                    ax.plot(dist_arr, spd_kmh_arr, ls='--', label=offset_str)

                ax.set_xlabel("Distance (m)", fontsize=9)
                ax.set_ylabel("Speed (km/h)", fontsize=9)
                ax.grid(True)
                ax.legend(fontsize=8)

        fig.suptitle("Speed (km/h) vs. Distance (m)\n(±10/20 km/h, Dry/Wet, Alert/Tired)", fontsize=13)
        plt.tight_layout()
        plt.show()

    else:
        # Simpler friction approach (no drag)
        dist_total, dist_brake, dist_react = stopping_distance_simple_friction(
            speed_kmh, mu_final, slope_val, reaction_time
        )
        if dist_total == float('inf'):
            print("\nCar cannot stop under these conditions (net deceleration <= 0).")
            return
        print("\n=== RESULTS (SIMPLE FRICTION) ===")
        print(f"Reaction distance: {dist_react:.2f} m")
        print(f"Braking distance:  {dist_brake:.2f} m")
        print(f"TOTAL distance:    {dist_total:.2f} m")
        print("NOTE: No drag included; results approximate.")

    # Ask user to run again or exit
    go_again = input("\nGo again? (y/n): ").strip().lower()
    start_over = (go_again == "y")
    if start_over:
        main()
    else:
        print("\nDone. Thank you for using the Stopping Distance Calculator!")

if __name__ == "__main__":
    main()
