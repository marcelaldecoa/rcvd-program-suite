/* rcvd/capi/rcvd.h — the extern "C" ABI, spec §4.1.
 *
 * This is the boundary the Python, C# and MATLAB bindings consume. It is C, not
 * C++: no templates, no exceptions, no std types, no layout that depends on a
 * C++ compiler.
 *
 * ERROR MODEL (§4.6 Rule A3). The core returns Result<T> and NEVER throws across
 * an API boundary, so nothing here wraps a call in try/catch — there is nothing
 * to catch. Result<T> cannot cross a C boundary, so it is decomposed into a
 * status code plus a thread-local last-error code and message, which is the
 * standard C spelling of the same thing.
 *
 * Every function is declared noexcept on the C++ side. The only escapes §4.6
 * permits are unrecoverable programming defects (std::bad_alloc, a failed
 * assertion), and terminating on those is the honest behaviour at a C boundary
 * rather than letting an exception unwind through it.
 *
 * ALL VALUES ARE SI (Rule U1): metre, kilogram, second, newton, radian. Use
 * rcvd_convert() at the caller's boundary; that is Rule U2's second crossing.
 */
#ifndef RCVD_CAPI_RCVD_H
#define RCVD_CAPI_RCVD_H

#ifdef __cplusplus
extern "C" {
/* Every entry point is noexcept: §4.6 permits an escape only for an
 * unrecoverable programming defect, and terminating at the boundary is the
 * honest behaviour rather than unwinding a C++ exception through C. The header
 * itself stays valid C, so the keyword is spelled through a macro. */
#  define RCVD_NOEXCEPT noexcept
#else
#  define RCVD_NOEXCEPT
#endif

typedef enum rcvd_status {
    RCVD_OK = 0,
    RCVD_ERROR = 1
} rcvd_status;

/* ---- version and provenance --------------------------------------------- */
void        rcvd_version(int* major, int* minor, int* patch) RCVD_NOEXCEPT;
const char* rcvd_spec_revision(void) RCVD_NOEXCEPT;
/* The ABI revision of THIS header. Bindings should check it at load time. */
int         rcvd_abi_version(void) RCVD_NOEXCEPT;

/* ---- errors -------------------------------------------------------------- */
/* Valid until the next failing call on the same thread. Empty after success. */
const char* rcvd_last_error_code(void) RCVD_NOEXCEPT;
const char* rcvd_last_error_message(void) RCVD_NOEXCEPT;

/* ---- units (Rule U2) ----------------------------------------------------- */
/* Convert between two units of the SAME dimension, both from §5's closed
 * vocabulary. Returns RCVD_ERROR if either unit is unknown or if the two belong
 * to different dimensions — there is no silent pass-through. */
rcvd_status rcvd_convert(double value, const char* from_unit, const char* to_unit,
                         double* out) RCVD_NOEXCEPT;

/* ---- vehicle ------------------------------------------------------------- */
typedef struct rcvd_vehicle rcvd_vehicle;

rcvd_status rcvd_vehicle_load_json(const char* json, rcvd_vehicle** out) RCVD_NOEXCEPT;
void        rcvd_vehicle_free(rcvd_vehicle* v) RCVD_NOEXCEPT;

/* Resolved SI properties by name: "m", "I_z", "a", "b", "l", "h", "m_s", "h_s",
 * "a_s", "t_F", "t_R", "W", "W_F", "W_R", "g", "steering_ratio",
 * "front_weight_fraction", "roll_gradient_deg_per_g". String-keyed so that
 * adding a field does not break the ABI. */
rcvd_status rcvd_vehicle_get(const rcvd_vehicle* v, const char* field, double* out) RCVD_NOEXCEPT;

/* Warnings raised while resolving (W-VEH-120, W-VEH-141). */
int         rcvd_vehicle_warning_count(const rcvd_vehicle* v) RCVD_NOEXCEPT;
const char* rcvd_vehicle_warning_code(const rcvd_vehicle* v, int index) RCVD_NOEXCEPT;
const char* rcvd_vehicle_warning_message(const rcvd_vehicle* v, int index) RCVD_NOEXCEPT;

/* ---- P1, the bicycle model (§8) ------------------------------------------ */
/* Cornering stiffnesses are AXLE values in N/rad and must be negative (§3.3). */

typedef struct rcvd_derivatives {
    double Y_beta;   /* N/rad       */
    double Y_r;      /* N*s/rad     */
    double Y_delta;  /* N/rad       */
    double N_beta;   /* N*m/rad     */
    double N_r;      /* N*m*s/rad   */
    double N_delta;  /* N*m/rad     */
} rcvd_derivatives;

typedef struct rcvd_handling {
    double K;          /* s^2/m^2 */
    double K_us;       /* rad per (m/s^2) */
    double NSP, SM;
    double V_tan;      /* m/s */
    double V_ch;       /* m/s; 0 and has_V_ch == 0 when the vehicle oversteers */
    double V_crit;     /* m/s; 0 and has_V_crit == 0 when it understeers */
    int    has_V_ch, has_V_crit;
} rcvd_handling;

typedef struct rcvd_steady {
    double v;       /* m/s   */
    double r;       /* rad/s */
    double beta;    /* rad   */
    double A_Y;     /* m/s^2 */
    double rcond;
} rcvd_steady;

typedef struct rcvd_eigen {
    double omega_n, zeta;
    double re1, im1, re2, im2;
    double tau_dominant, trace, det;
    int    stable, oscillatory;
} rcvd_eigen;

rcvd_status rcvd_stability_derivatives(const rcvd_vehicle* v, double CF, double CR,
                                       double V, rcvd_derivatives* out) RCVD_NOEXCEPT;
rcvd_status rcvd_handling_metrics(const rcvd_vehicle* v, double CF, double CR,
                                  rcvd_handling* out) RCVD_NOEXCEPT;
rcvd_status rcvd_steady_state(const rcvd_vehicle* v, double CF, double CR, double V,
                              double delta, rcvd_steady* out) RCVD_NOEXCEPT;
rcvd_status rcvd_eigenstructure(const rcvd_vehicle* v, double CF, double CR, double V,
                                rcvd_eigen* out) RCVD_NOEXCEPT;
/* (8.15) yaw-rate gain, the quantity identity I-2 checks three ways. */
rcvd_status rcvd_yaw_rate_gain(const rcvd_vehicle* v, double CF, double CR, double V,
                               double* out) RCVD_NOEXCEPT;

/* ---- P6, ride and roll rates (§12) --------------------------------------- */
typedef struct rcvd_ride_roll_components {
    double tire_rate_per_tire;    /* N/m; pass a negative value for rigid tires */
    double wheel_track;           /* m   */
    double spring_track;          /* m   */
    double wheel_centre_rate;     /* N/m, per wheel */
    double aux_roll_stiffness;    /* N*m/rad */
    double axle_weight;           /* N   */
} rcvd_ride_roll_components;

typedef struct rcvd_ride_roll_totals {
    double ride_rate;                     /* N/m */
    double roll_stiffness_tire;           /* N*m/rad */
    double roll_stiffness_springs;        /* N*m/rad */
    double roll_stiffness_chassis_to_wc;  /* N*m/rad */
    double roll_stiffness_total;          /* N*m/rad, chassis to ground */
    double ride_frequency_Hz;
} rcvd_ride_roll_totals;

typedef struct rcvd_ride_roll_request {
    double desired_ride_rate;       /* N/m */
    double desired_roll_stiffness;  /* N*m/rad, chassis to ground */
    double tire_rate_per_tire;      /* N/m; negative for rigid */
    double wheel_track, spring_track, axle_weight;
} rcvd_ride_roll_request;

/* Direction A, (12.7). */
rcvd_status rcvd_ride_roll_components_to_totals(const rcvd_ride_roll_components* in,
                                                rcvd_ride_roll_totals* out) RCVD_NOEXCEPT;
/* Direction B, (12.8). Exact inverse of the above (invariant I-5). */
rcvd_status rcvd_ride_roll_totals_to_components(const rcvd_ride_roll_request* in,
                                                rcvd_ride_roll_components* out) RCVD_NOEXCEPT;

/* ---- P4, the lap solver (§11) -------------------------------------------- */
typedef struct rcvd_lap {
    int    event;          /* 0 none, 1 lap, 2 wall_in, 3 wall_out, 4 spin, 5 timeout */
    double time;           /* s */
    long   steps_accepted, steps_rejected;
    int    hint_fired;     /* H-SPD-001 (§11.6) */
} rcvd_lap;

rcvd_status rcvd_lap_solve_bicycle(const rcvd_vehicle* v, double CF, double CR, double V,
                                   double delta, double inner_radius, double outer_radius,
                                   double start_radius, rcvd_lap* out) RCVD_NOEXCEPT;

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* RCVD_CAPI_RCVD_H */
