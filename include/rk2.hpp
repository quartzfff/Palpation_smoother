#include <Eigen/Dense>

Eigen::VectorXd f(double t, const Eigen::VectorXd& x)
{
    return -x;
}

Eigen::VectorXd rk2_step(double t, const Eigen::VectorXd& x, double h){
    Eigen::VectorXd k1 = f(t,x);
    Eigen::VectorXd k2 = f(t+0.5*h, x+0.5*h*k1);
    return x+h*k2;
}


class rk2_step(
    const RHS& f,
    double t,
    const Vector& y,
    double h)
    {
        Vector one_step(double t,const Vector& y){
        Vector k1 = f(t,y);
        Vector k2 = f;
        return y+h*k2;
        }

    }
