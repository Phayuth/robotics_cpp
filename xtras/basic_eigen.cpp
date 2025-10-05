#include <Eigen/Dense>
#include <iostream>

// Function to demonstrate Eigen version information
void demonstrateEigenVersion() {
    std::cout << "=== Eigen Version Information ===" << std::endl;
    std::cout << "Eigen version: " << EIGEN_WORLD_VERSION << "."
              << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
    std::cout << std::endl;
}

// Function to demonstrate vector initialization
void demonstrateVectorInitialization() {
    std::cout << "=== Vector Initialization ===" << std::endl;

    // Vector with initialized values
    Eigen::Vector3d a(0, 1, 2); // allocated vector with 3 dim
    // allocated vector with 4 dim with initializer list
    Eigen::Vector4i b = {1, 3, 2, 3};
    // allocated vector with 2 dim from constructor
    Eigen::Vector2d c = Eigen::Vector2d(1, 2);

    std::cout << "Vector3d a(0,1,2): " << a.transpose() << std::endl;
    std::cout << "Vector4i b{1,3,2,3}: " << b.transpose() << std::endl;
    std::cout << "Vector2d c(1,2): " << c.transpose() << std::endl;
    std::cout << "Vector a properties - Rows: " << a.rows()
              << ", Cols: " << a.cols() << std::endl;
    std::cout << std::endl;
}

// Function to demonstrate dynamic vector operations
void demonstrateDynamicVectors() {
    std::cout << "=== Dynamic Vector Operations ===" << std::endl;

    // Vector with uninitialized values
    Eigen::VectorXd d(10); // dynamic size vector, allocated with arbitrary dim
    Eigen::VectorXd e(3);

    // Assigning values using << operator
    d << 1.2, 21.2, 23.3, 1.2, 21.2, 23.3, 52.0, 1.2, 21.2, 23.3;
    std::cout << "Dynamic vector d assigned with << operator:" << std::endl;
    std::cout << d.transpose() << std::endl;

    // Assigning values using () operator
    e(0) = 2.3;
    e(1) = 1.4;
    e(2) = 5.1;
    std::cout << "Vector e assigned with () operator:" << std::endl;
    std::cout << e.transpose() << std::endl;
    std::cout << std::endl;
}

// Function to demonstrate vector transpose operations
void demonstrateVectorTranspose() {
    std::cout << "=== Vector Transpose Operations ===" << std::endl;

    Eigen::Vector3d a(0, 1, 2);

    Eigen::RowVector3d aTranspose1 = a;             // implicit conversion
    Eigen::RowVector3d aTranspose2 = a.transpose(); // explicit transpose
    auto aTranspose3 = a.transpose();               // auto type deduction

    std::cout << "Original vector a: " << a.transpose() << std::endl;
    std::cout << "Transpose (implicit): " << aTranspose1 << std::endl;
    std::cout << "Transpose (explicit): " << aTranspose2 << std::endl;
    std::cout << "Transpose (auto): " << aTranspose3 << std::endl;
    std::cout << "Transpose properties - Rows: " << aTranspose3.rows()
              << ", Cols: " << aTranspose3.cols() << std::endl;
    std::cout << std::endl;
}

// Function to demonstrate special vector creation
void demonstrateSpecialVectors() {
    std::cout << "=== Special Vector Creation ===" << std::endl;

    // Random vector
    Eigen::VectorXd randomVec = Eigen::VectorXd::Random(3);
    std::cout << "Random vector (3D):" << std::endl;
    std::cout << randomVec.transpose() << std::endl;

    // Ones vector
    Eigen::VectorXd onesVec(5);
    onesVec.setOnes();
    std::cout << "Ones vector (5D):" << std::endl;
    std::cout << onesVec.transpose() << std::endl;

    // Zero vector
    Eigen::VectorXd zeroVec = Eigen::VectorXd::Zero(4);
    std::cout << "Zero vector (4D):" << std::endl;
    std::cout << zeroVec.transpose() << std::endl;
    std::cout << std::endl;
}

// Function to demonstrate matrix initialization
void demonstrateMatrixInitialization() {
    std::cout << "=== Matrix Initialization ===" << std::endl;

    // Identity matrix
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    std::cout << "3x3 Identity matrix A:" << std::endl;
    std::cout << A << std::endl;

    // Fixed size matrix
    Eigen::Matrix<double, 2, 3> B;
    B << 2, 3, 4, 5, 6, 7;
    std::cout << "2x3 Matrix B assigned with << operator:" << std::endl;
    std::cout << B << std::endl;

    // Dynamic matrix
    Eigen::MatrixXd C(3, 3);
    C << 2, 3, 4, 5, 6, 7, 8, 9, 1;
    std::cout << "3x3 Dynamic matrix C:" << std::endl;
    std::cout << C << std::endl;
    std::cout << std::endl;
}

// Function to demonstrate matrix operations
void demonstrateMatrixOperations() {
    std::cout << "=== Matrix Operations ===" << std::endl;

    Eigen::Matrix<double, 2, 3> B;
    B << 2, 3, 4, 5, 6, 7;

    std::cout << "Original matrix B:" << std::endl;
    std::cout << B << std::endl;

    std::cout << "Matrix B * 2 (scalar multiplication):" << std::endl;
    std::cout << B * 2 << std::endl;

    std::cout << "Element-wise product B.cwiseProduct(B * 2):" << std::endl;
    std::cout << B.cwiseProduct(B * 2) << std::endl;
    std::cout << std::endl;
}

// Function to demonstrate special matrix creation
void demonstrateSpecialMatrices() {
    std::cout << "=== Special Matrix Creation ===" << std::endl;

    // Zero matrix
    Eigen::MatrixXd zeroMat = Eigen::MatrixXd::Zero(3, 3);
    std::cout << "3x3 Zero matrix:" << std::endl;
    std::cout << zeroMat << std::endl;

    // Ones matrix
    Eigen::MatrixXd onesMat = Eigen::MatrixXd::Ones(2, 4);
    std::cout << "2x4 Ones matrix:" << std::endl;
    std::cout << onesMat << std::endl;

    // Random matrix
    Eigen::MatrixXd randomMat = Eigen::MatrixXd::Random(2, 2);
    std::cout << "2x2 Random matrix:" << std::endl;
    std::cout << randomMat << std::endl;
    std::cout << std::endl;
}

int main() {
    // Call individual demonstration functions
    demonstrateEigenVersion();
    demonstrateVectorInitialization();
    demonstrateDynamicVectors();
    demonstrateVectorTranspose();
    demonstrateSpecialVectors();
    demonstrateMatrixInitialization();
    demonstrateMatrixOperations();
    demonstrateSpecialMatrices();

    std::cout << "=== All Eigen Basic Demonstrations Complete ===" << std::endl;
    return 0;
}