#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

using namespace std;

// Vertex: optimization variable (parameters a, b, c)
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d(update);
    }

    virtual bool read(istream& in) override { return true; }
    virtual bool write(ostream& out) const override { return true; }
};

// Edge: error model
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x) : _x(x) {}

    void computeError() override {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - exp(abc(0) * _x * _x + abc(1) * _x + abc(2));
    }

    virtual bool read(istream& in) override { return true; }
    virtual bool write(ostream& out) const override { return true; }

public:
    double _x;
};

int main(int argc, char** argv) {
    // Real parameters
    double a = 1.0, b = 2.0, c = 1.0;
    int N = 100; // data points
    double w_sigma = 1.0; // noise
    cv::RNG rng;

    vector<double> x_data, y_data;
    x_data.reserve(N);
    y_data.reserve(N);

    cout << "Generating data:" << endl;
    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        cout << x_data[i] << " " << y_data[i] << endl;
    }

    // Create graph optimizer
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block; // 3 params, 1 error
    typedef g2o::LinearSolverDense<Block::PoseMatrixType> LinearSolver;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<Block>(std::make_unique<LinearSolver>())
    );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // Add vertex
    auto* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0, 0, 0));
    v->setId(0);
    optimizer.addVertex(v);

    // Add edges
    for (int i = 0; i < N; ++i) {
        CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1.0 / (w_sigma * w_sigma));
        optimizer.addEdge(edge);
    }

    // Optimize
    cout << "Starting optimization..." << endl;
    auto t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    auto t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Optimization finished in " << time_used.count() << " seconds." << endl;

    // Output result
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "Estimated model: " << abc_estimate.transpose() << endl;

    return 0;
}

