/*******************************************************************************
Purpose:
  Targeting algorithm that manages a hemispherical coverage matrix and
  selects the next target to image based on uncovered az/el bins.

  Azimuth and Elevation are computed in the LVLH (Local Vertical / Local
  Horizontal) frame:
    - Origin: spacecraft position
    - -Z (Nadir): defines El = 90° (directly at Earth)
    - +X (ram / velocity direction): Azimuth reference (Az = 0°)
    - +Y: completes right-hand frame (Az = 90° starboard)

  Coverage matrix: [36 az bins][9 el bins]
    - Az  : 0–360° in 10° steps  → index = floor(az_deg / 10)
    - El  : 0– 90° in 10° steps  → index = floor(el_deg / 10)
    - El = 0–10° is near-horizon; El = 80–90° is near-nadir.

Library dependencies:
  ((../src/targeting.cpp))
*******************************************************************************/
#pragma once

#include <cstring>
#include <cmath>

namespace gnc {

static constexpr int  AZ_BINS   = 36;   // 360° / 10°
static constexpr int  EL_BINS   =  9;   //  90° /  10°
static constexpr double AZ_STEP = 10.0; // degrees
static constexpr double EL_STEP = 10.0; // degrees

struct TargetAzEl {
    double az_deg; //!< trick_units(deg)  [0, 360)
    double el_deg; //!< trick_units(deg)  [0,  90]
    bool   visible; //!< trick_units(--)
};

class TargetingAlgorithm {
public:
    // ---------------------------------------------------------------
    // Coverage state
    // ---------------------------------------------------------------
    /** coverage_matrix[az_idx][el_idx] = true when bin has been imaged. */
    bool coverage_matrix[AZ_BINS][EL_BINS]; //!< trick_units(--)

    /** Last-computed az/el for each target (size = max_targets). */
    static constexpr int max_targets = 10;
    TargetAzEl target_azel[max_targets]; //!< trick_units(--)

    // ---------------------------------------------------------------
    // Selection output
    // ---------------------------------------------------------------
    /** Index (0-based) of the currently selected target. -1 = none. */
    int selected_target_idx; //!< trick_units(--)

    /** Estimated az/el bin that the selected target scores best for. */
    int selected_az_bin; //!< trick_units(--)
    int selected_el_bin; //!< trick_units(--)

    /** Count of bins imaged so far. */
    int bins_imaged; //!< trick_units(--)

    /** Total bins in the matrix (AZ_BINS * EL_BINS). */
    int total_bins; //!< trick_units(--)

    /** Minimum elevation (deg) to consider a target visible. */
    double min_elevation_deg; //!< trick_units(deg)

    /** True when all bins have been imaged. */
    bool coverage_complete; //!< trick_units(--)

    // ---------------------------------------------------------------
    // Imaging dwell tracking
    // ---------------------------------------------------------------
    /** Seconds the current target must be tracked before marking imaged. */
    double image_dwell_time_s; //!< trick_units(s)

    /** Accumulated dwell on the current target. */
    double current_dwell_s; //!< trick_units(s)

    /** True while we are actively dwelling on a selected target. */
    bool dwelling; //!< trick_units(--)

    // ---------------------------------------------------------------
    // Methods
    // ---------------------------------------------------------------
    TargetingAlgorithm();

    /**
     * Compute LVLH-relative azimuth and elevation for all targets.
     *
     * @param sc_pos_eci  Spacecraft position in ECI (m)
     * @param sc_vel_eci  Spacecraft velocity in ECI (m/s)
     * @param target_eci  Array of target ECI positions, shape [N][3] (m)
     * @param num_targets Number of valid entries in target_eci
     */
    void compute_azel(const double sc_pos_eci[3],
                      const double sc_vel_eci[3],
                      const double target_eci[][3],
                      int          num_targets);

    /**
     * Select the best un-imaged target to track next.
     * Returns the index (0-based) of the selected target, or -1 if none.
     */
    int select_target(int num_targets);

    /**
     * Accumulate dwell time; mark current target bins imaged when ready.
     *
     * @param dt    Time step (s)
     * @returns true when the selected target bins were just marked as imaged.
     */
    bool update_dwell(double dt);

    /**
     * Force-mark the az/el bin associated with target i as imaged.
     */
    void mark_imaged(int target_idx);

    /** Reset all coverage data. */
    void reset_coverage();

    /** Returns true if a given az/el pair is in an already-imaged bin. */
    bool is_bin_imaged(double az_deg, double el_deg) const;

private:
    static constexpr double EARTH_RADIUS_M = 6378137.0;

    /**
     * Check if the line-of-sight from spacecraft to target is obstructed by Earth.
     */
    bool is_obstructed(const double sc_pos_eci[3], const double target_eci[3]) const;

    /** Convert az/el degrees to bin indices; returns false if out of range. */
    static bool to_bin(double az_deg, double el_deg, int& az_idx, int& el_idx);
};

} // namespace gnc
