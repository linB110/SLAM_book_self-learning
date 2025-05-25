#include <iostream>
#include <ctime>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char** argv)
{
    // 2x3 matrix
    Eigen::Matrix<float, 2, 3> mat23;
    
    // vector = matrix<double>
    Eigen::Vector3d v_3d;
    
    // 3x3 matrix and initialize with 0
    Eigen::Matrix3d mat33 = Eigen::Matrix3d::Zero();
    
    // dynamic size matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> max_Dynamic;
    
    mat23 << 1, 2, 3, 4, 5, 6;
    cout << mat23 << endl;
    
    for (int i = 0; i < 1; i++){
        for (int j = 0; j < 2; j++){
            cout << mat23(i, j) << endl;
        }
    }
    
    v_3d << 3, 2, 1;
    Eigen::Matrix<double, 2, 1> result = mat23.cast<double>() * v_3d;
    cout << result << endl;
    
    mat33 = Eigen::Matrix3d::Random();
    cout << mat33 << endl << endl;
    
    cout << mat33.transpose() << endl;   // transpose 
    cout << mat33.sum() << endl;         // sum of elements
    cout << 10*mat33 << endl;            // multiply bt a number
    cout << mat33.inverse() << endl;     // inverse matrix
    cout << mat33.determinant() << endl; // determinant
    
    // eigen value
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver ( mat33.transpose() * mat33 );
    cout << "eigen valure =  " << eigen_solver.eigenvalues() << endl;
    cout << "eigen vector = " << eigen_solver.eigenvectors() << endl; 
    
    Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE > matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    Eigen::Matrix< double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random( MATRIX_SIZE,1 );
    
    clock_t time_stt = clock();
    
    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse()*v_Nd;
    cout <<"time use in normal invers is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;
    
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout <<"time use in Qr compsition is " <<1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
    
    return 0;
}
