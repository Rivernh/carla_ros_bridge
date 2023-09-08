#include<lattice_planner/Spline.h>

QuinticPolynomial::QuinticPolynomial(double xs,double vs,double as,double xe, double ve, double ae, double time)
{
    a0 = xs;
    a1 = vs;
    a2 = as / 2;
    Matrix3d A;
    Vector3d b;
    Vector3d x;
    A<<pow(time,3),pow(time,4),pow(time,5),
            3*pow(time,2),4*pow(time,3),5*pow(time,4),
            6*time,12*pow(time,2),20*pow(time,3);
    b<<xe-a0-a1*time-a2*pow(time,2),
            ve-a1-2*a2*time,
            ae-2*a2;
    x = A.colPivHouseholderQr().solve(b);
    a3 = x(0);
    a4 = x(1);
    a5 = x(2);
}

QuinticPolynomial::~QuinticPolynomial()
{
    cout<<"deleted!"<<endl;
}

double QuinticPolynomial::calc_point(double t)
{
    double xt;
    xt = a0 + a1*t + a2 * pow(t,2)  + a3 * pow(t,3) +
             a4 * pow(t,4) + a5 * pow(t,5);
    return xt;
}

double QuinticPolynomial::calc_first_derivative(double t)
{
    double xt;
    xt = a1+ 2*a2 * t  + 3*a3 * pow(t,2) + 4*a4 * pow(t,3) + 5 *a5 * pow(t,4);
    return xt;
}

double QuinticPolynomial::calc_second_derivative(double t)
{
    double xt;
    xt = 2*a2  + 6*a3 * t + 12*a4 * pow(t,2) + 20 * a5 * pow(t,3);
    return xt;
}

double QuinticPolynomial::calc_third_derivative(double t)
{
    double xt;
    xt =  6*a3  + 24*a4 * t + 60 * a5 * pow(t,2);
    return xt;
}