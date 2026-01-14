#include <iostream>
#include <Eigen/Dense>


class Point3{
    private:
        Eigen::Vector3d p_;
    public:
        Point3(Eigen::Vector3d& p): p_(p){
        }

        //double x() const { return x_; }
        void printp() const{std::cout << p_;}

};



int main(){
    // Eigen::Vector3d v; //3x1
    // Eigen::Matrix3d R; //3x1
    // Eigen::VectorXd x(6);
    // Eigen::MatrixXd A(6,6);
    // Eigen::Vector3d v(1,2,3);
    // Eigen::VectorXd x(3);
    // x <<1.0,2.0,3.0;
    // Eigen::Matrix3d R;
    // R << 1, 0, 0,
    //     0, 1, 0,
    //     0, 0, 1;
    // Eigen::Matrix3d I =Eigen::Matrix3d::Identity();
    // Eigen::Vector3d z = Eigen::Vector3d::Zero();
    // double dot = a.dot(b);void printNorm(const Eigen::Vector3d& v){
    //     std::cout<<v.norm()<<std::endl;
    // }
    Eigen::Vector3d v(1.0,2.0,3.0);
    Point3 point1(v);
    point1.printp();
    return 0;

}