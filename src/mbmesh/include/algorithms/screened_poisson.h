#pragma once

#include <algorithm>
#include <string>

#include "../data_types/geometry.h"

// How the grid equation is solved; chosen with --poisson-solver.
enum class PoissonSolver {
    Jacobi,   // plain Jacobi relaxation
};

[[nodiscard]] const char *poisson_solver_name(PoissonSolver solver);

// The names --poisson-solver accepts, for example "jacobi", for help and error messages.
[[nodiscard]] const char *poisson_solver_names();

// Sets *solver and returns true if name is one of poisson_solver_names().
[[nodiscard]] bool parse_poisson_solver(const std::string &name, PoissonSolver *solver);

struct ScreenedPoissonOptions {
    // Regular-grid spacing in point-cloud coordinate units. This is the main
    // reconstruction resolution and memory-control parameter.
    double cell_size = 0.25;

    // Empty margin around the sample bounds. At least a few cells are needed
    // for the implicit field to transition before reaching the grid boundary.
    double padding = 0.75;

    // Radius over which each oriented sample contributes to grid nodes.
    // Approximately two grid cells gives continuous support without excessive
    // smoothing for the default resolution.
    double normal_splat_radius = 0.5;

    // Strength of positional constraints at observed samples. Larger values
    // adhere more closely to samples but can reproduce more measurement noise.
    double screening_weight = 4.0;

    // The method that solves the grid equation.
    PoissonSolver solver = PoissonSolver::Jacobi;

    // Number of relaxation passes used to solve the grid equation (a Jacobi
    // sweep, or whatever one pass of the chosen solver is). More iterations
    // improve convergence at a proportional runtime cost.
    int solver_iterations = 200;

    // Apply sample-position constraints in addition to the normal field.
    bool use_screening = true;

    // When set, the grid covers domain_bounds exactly instead of the padded
    // bounds of the samples. Chunked reconstruction uses this so that every
    // chunk's grid nodes sit on one global lattice.
    bool use_fixed_domain = false;
    Bounds3D domain_bounds;

    // Scalar value assigned to the observed surface after centering the field.
    // MarchingCubesOptions::iso_value should normally use the same value.
    double iso_value = 0.0;
};

/******************************************************************************/

// What screened_poisson() reports about one solve, to show where the time goes
// and how far the solver got.
struct PoissonDiagnostics {
    PoissonSolver solver = PoissonSolver::Jacobi;
    long long cells = 0;
    int iterations = 0;
    double splat_seconds = 0.0;   // splatting the normals and samples, and the right-hand side
    double solve_seconds = 0.0;   // the iterations, without measuring the residual

    // How far the solution is from solving the grid equation: the 2-norm of
    // b - A u over the interior cells divided by the 2-norm of b. It is 1 at the
    // start (u = 0) and falls toward 0 as the solver converges. Negative when
    // not measured.
    double residual_midway = -1.0;   // after half the iterations
    double residual_final = -1.0;
};

/******************************************************************************
 * Extracts an implicit surface from an OrientedPoint set. diagnostics, if
 * given, is filled in.
 ******************************************************************************/

ScalarGrid3D screened_poisson(const OrientedPointCloud &oriented_points, ScreenedPoissonOptions options,
                              PoissonDiagnostics *diagnostics = nullptr);
