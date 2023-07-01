#include "eigen_01_point_loading.h"
#include <iostream>
#include <fstream>
#include <list>

using namespace std;

using Vector3fList = std::list<Vector3f, Eigen::aligned_allocator<Vector3f> >;

int main(int argc, char** argv) {
    if (argc<2) return -1;

    ifstream is(argv[1]);
    //ifstream is("test_data/points_3d.dat");

    Vector3fList points3D;
    int dimension = 3;
    int num_points = loadPoints(points3D, is, dimension);

    Vector3f mean, mu;
    Matrix3f cov;
    mean.setZero();
    mu.setZero();
    cov.setZero();

    cerr << "I read " << num_points * dimension << " points from the stream." << endl;

    // Mean
    for (const auto& v : points3D) {
        mean += v;
    }

    mu = mean / (num_points * dimension);

    cerr << "-------------------------" << endl;
    std::cout << "mu: \n" << mu << endl;
    cerr << "-------------------------" << endl;

    // Covariance
    for (const auto& v : points3D) {
        cov += (v - mu) * (v - mu).transpose();
    }

    cov /= ((num_points * dimension) - 1);

    std::cout << "cov: \n" << cov << endl;

}
