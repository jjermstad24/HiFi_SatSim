
#include "../include/targeting.hh"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace gnc {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
TargetingAlgorithm::TargetingAlgorithm()
{
    reset_coverage();
    selected_target_idx = -1;
    selected_az_bin     = -1;
    selected_el_bin     = -1;
    bins_imaged         =  0;
    total_bins          =  AZ_BINS * EL_BINS;
    min_elevation_deg   =  5.0;
    coverage_complete   =  false;
    image_dwell_time_s  = 30.0;
    current_dwell_s     =  0.0;
    dwelling            =  false;
    std::memset(target_azel, 0, sizeof(target_azel));
}

// ---------------------------------------------------------------------------
// reset_coverage
// ---------------------------------------------------------------------------
void TargetingAlgorithm::reset_coverage()
{
    std::memset(coverage_matrix, 0, sizeof(coverage_matrix));
    bins_imaged       = 0;
    coverage_complete = false;
}

// ---------------------------------------------------------------------------
// to_bin  (static helper)
// ---------------------------------------------------------------------------
bool TargetingAlgorithm::to_bin(double az_deg, double el_deg,
                                 int& az_idx, int& el_idx)
{
    // Az wraps, El is [0, 90]
    if (el_deg < 0.0 || el_deg > 90.0) return false;

    // Normalise azimuth into [0, 360)
    while (az_deg <   0.0) az_deg += 360.0;
    while (az_deg >= 360.0) az_deg -= 360.0;

    az_idx = static_cast<int>(az_deg / AZ_STEP);
    el_idx = static_cast<int>(el_deg / EL_STEP);

    // Clamp to valid range
    if (az_idx < 0)        az_idx = 0;
    if (az_idx >= AZ_BINS) az_idx = AZ_BINS - 1;
    if (el_idx < 0)        el_idx = 0;
    if (el_idx >= EL_BINS) el_idx = EL_BINS - 1;

    return true;
}

// ---------------------------------------------------------------------------
// is_bin_imaged
// ---------------------------------------------------------------------------
bool TargetingAlgorithm::is_bin_imaged(double az_deg, double el_deg) const
{
    int az_idx, el_idx;
    if (!to_bin(az_deg, el_deg, az_idx, el_idx)) return false;
    return coverage_matrix[az_idx][el_idx];
}

// ---------------------------------------------------------------------------
// compute_azel
//
// LVLH frame construction:
//   x_lvlh = v_rel / |v_rel|          (ram / flight direction, Az = 0°)
//   z_lvlh = -r_eci / |r_eci|         (nadir-pointing, El = 90°)
//   y_lvlh = z_lvlh × x_lvlh          (starboard, Az = 90°)
//
// For each target, the LOS vector (in LVLH) is:
//   los_lvlh = T_lvlh_eci * (tgt_eci - sc_eci) normalised
//
// Elevation = arcsin(-los_z) for nadir-down convention.
// Azimuth   = atan2(los_y, los_x)
// ---------------------------------------------------------------------------
void TargetingAlgorithm::compute_azel(const double sc_pos_eci[3],
                                       const double sc_vel_eci[3],
                                       const double target_eci[][3],
                                       int          num_targets)
{
    constexpr double DEG = 180.0 / M_PI;

    // -- Build LVLH basis vectors (ECI) -----------------------------------
    double r_mag = std::sqrt(sc_pos_eci[0]*sc_pos_eci[0]
                           + sc_pos_eci[1]*sc_pos_eci[1]
                           + sc_pos_eci[2]*sc_pos_eci[2]);
    double v_mag = std::sqrt(sc_vel_eci[0]*sc_vel_eci[0]
                           + sc_vel_eci[1]*sc_vel_eci[1]
                           + sc_vel_eci[2]*sc_vel_eci[2]);

    if (r_mag < 1.0 || v_mag < 1.0) {
        // Degenerate – mark all invisible
        for (int i = 0; i < num_targets && i < max_targets; i++) {
            target_azel[i].visible = false;
        }
        return;
    }

    // x_lvlh: ram (velocity direction)
    double x_lvlh[3] = { sc_vel_eci[0]/v_mag,
                          sc_vel_eci[1]/v_mag,
                          sc_vel_eci[2]/v_mag };

    // z_lvlh: nadir (-r direction)
    double z_lvlh[3] = { -sc_pos_eci[0]/r_mag,
                          -sc_pos_eci[1]/r_mag,
                          -sc_pos_eci[2]/r_mag };

    // y_lvlh: cross(z, x)  →  starboard
    double y_lvlh[3] = {
        z_lvlh[1]*x_lvlh[2] - z_lvlh[2]*x_lvlh[1],
        z_lvlh[2]*x_lvlh[0] - z_lvlh[0]*x_lvlh[2],
        z_lvlh[0]*x_lvlh[1] - z_lvlh[1]*x_lvlh[0]
    };

    // -- Per-target az/el -------------------------------------------------
    for (int i = 0; i < num_targets && i < max_targets; i++) {
        // LOS in ECI
        double los_eci[3] = {
            target_eci[i][0] - sc_pos_eci[0],
            target_eci[i][1] - sc_pos_eci[1],
            target_eci[i][2] - sc_pos_eci[2]
        };
        double los_mag = std::sqrt(los_eci[0]*los_eci[0]
                                 + los_eci[1]*los_eci[1]
                                 + los_eci[2]*los_eci[2]);
        if (los_mag < 1.0) {
            target_azel[i].visible = false;
            continue;
        }
        los_eci[0] /= los_mag;
        los_eci[1] /= los_mag;
        los_eci[2] /= los_mag;

        // Project onto LVLH axes
        double lx = x_lvlh[0]*los_eci[0] + x_lvlh[1]*los_eci[1] + x_lvlh[2]*los_eci[2];
        double ly = y_lvlh[0]*los_eci[0] + y_lvlh[1]*los_eci[1] + y_lvlh[2]*los_eci[2];
        double lz = z_lvlh[0]*los_eci[0] + z_lvlh[1]*los_eci[1] + z_lvlh[2]*los_eci[2];

        // Clamp for asin
        if (lz >  1.0) lz =  1.0;
        if (lz < -1.0) lz = -1.0;

        // Elevation: angle from the LVLH x-y plane toward nadir (-z).
        // A target directly at nadir has lz = -1  →  el = 90°.
        double el_deg = std::asin(-lz) * DEG;

        // Azimuth: angle in the x-y plane from +x (ram), measured right-hand.
        double az_deg = std::atan2(ly, lx) * DEG;
        if (az_deg < 0.0) az_deg += 360.0;

        target_azel[i].az_deg  = az_deg;
        target_azel[i].el_deg  = el_deg;
        target_azel[i].visible = (el_deg >= min_elevation_deg);
    }
}

// ---------------------------------------------------------------------------
// select_target
//
// Strategy:
//   1. Only consider visible targets (el >= min_elevation_deg).
//   2. Prefer targets whose az/el bin has NOT yet been imaged.
//   3. Among those, pick the one with the highest elevation (best signal,
//      least horizon interference, and longest upcoming access window).
//   4. If all visible targets are already imaged, pick the one with highest
//      elevation still (so the spacecraft doesn't go idle).
// ---------------------------------------------------------------------------
int TargetingAlgorithm::select_target(int num_targets)
{
    if (coverage_complete) return -1;

    int    best_idx      = -1;
    bool   best_unimaged =  false;
    double best_el       = -1e30;

    int clamped = (num_targets < max_targets) ? num_targets : max_targets;

    for (int i = 0; i < clamped; i++) {
        const TargetAzEl& ta = target_azel[i];
        if (!ta.visible) continue;

        bool unimaged = !is_bin_imaged(ta.az_deg, ta.el_deg);

        // Accept if:
        //   (a) this is our first candidate at all, or
        //   (b) this target is unimaged and the previous best is imaged, or
        //   (c) both have the same imaged-status but this has higher elevation.
        bool take = false;
        if (best_idx < 0) {
            take = true;
        } else if (unimaged && !best_unimaged) {
            take = true;
        } else if (unimaged == best_unimaged && ta.el_deg > best_el) {
            take = true;
        }

        if (take) {
            best_idx      = i;
            best_unimaged = unimaged;
            best_el       = ta.el_deg;
        }
    }

    return best_idx;
}

// ---------------------------------------------------------------------------
// update_dwell
// ---------------------------------------------------------------------------
bool TargetingAlgorithm::update_dwell(double dt)
{
    if (!dwelling || selected_target_idx < 0) return false;

    current_dwell_s += dt;

    if (current_dwell_s >= image_dwell_time_s) {
        mark_imaged(selected_target_idx);
        current_dwell_s = 0.0;
        dwelling        = false;
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// mark_imaged
// ---------------------------------------------------------------------------
void TargetingAlgorithm::mark_imaged(int target_idx)
{
    if (target_idx < 0 || target_idx >= max_targets) return;

    int az_idx, el_idx;
    if (!to_bin(target_azel[target_idx].az_deg,
                target_azel[target_idx].el_deg,
                az_idx, el_idx)) return;

    if (!coverage_matrix[az_idx][el_idx]) {
        coverage_matrix[az_idx][el_idx] = true;
        bins_imaged++;

        std::printf("[Targeting] Bin imaged: az_bin=%d (%3.0f°–%3.0f°)  "
                    "el_bin=%d (%2.0f°–%2.0f°)  total=%d/%d\n",
                    az_idx,
                    az_idx * AZ_STEP, (az_idx + 1) * AZ_STEP,
                    el_idx,
                    el_idx * EL_STEP, (el_idx + 1) * EL_STEP,
                    bins_imaged,
                    total_bins);
    }

    if (bins_imaged >= total_bins) {
        coverage_complete = true;
        std::printf("[Targeting] *** FULL COVERAGE ACHIEVED (%d/%d bins) ***\n",
                    bins_imaged, total_bins);
    }
}

} // namespace gnc
