#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// Cost function for curve fitting
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    // Templated operator to compute residuals for auto-differentiation
    template <typename T>
    bool operator()(const T* const abc, T* residual) const
    {
        // The model: y = exp(ax^2 + bx + c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }

    const double _x, _y;  // Input data points
};

int main(int argc, char** argv)
{
    // Ground truth parameters for the model
    double a = 1.0, b = 2.0, c = 1.0;

    int N = 100;               // Number of data points
    double w_sigma = 1.0;      // Noise standard deviation
    cv::RNG rng;               // OpenCV random number generator
    double abc[3] = {0, 0, 0}; // Initial estimates for parameters a, b, c

    vector<double> x_data, y_data; // Containers for generated data

    cout << "Generating data:" << endl;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        double y = exp(a * x * x + b * x + c) + rng.gaussian(w_sigma); // Add Gaussian noise
        x_data.push_back(x);
        y_data.push_back(y);
        cout << x << " " << y << endl;
    }

    // Set up the Ceres problem
    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        // Each observation adds a residual block using automatic differentiation
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,  // No robust kernel
            abc       // Parameters to be optimized
        );
    }

    // Configure the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;        // Use dense QR decomposition
    options.minimizer_progress_to_stdout = true;         // Output progress to console

    ceres::Solver::Summary summary;
    auto t1 = chrono::steady_clock::now();               // Start timing
    ceres::Solve(options, &problem, &summary);           // Solve the optimization problem
    auto t2 = chrono::steady_clock::now();               // End timing

    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Solve time cost = " << time_used.count() << " seconds." << endl;

    // Output results
    cout << summary.BriefReport() << endl;
    cout << "Estimated parameters: a, b, c = ";
    for (auto a : abc)
        cout << a << " ";
    cout << endl;

    return 0;
}

