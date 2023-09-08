#include<lattice_planner/Spline.h>

QuarticPolynomial::QuarticPolynomial(double xs,double vs,double as,double ve, double ae, double time)
{
    a0 = xs;
    a1 = vs;
    a2 = as / 2;
    Matrix2d A;
    Vector2d b;
    Vector2d x;
    A<<3*pow(time,2),4*pow(time,4),
            6*time,12*pow(time,2);
    b<<ve-a1-2*a2*time,
            ae-2*a2;
    x = A.colPivHouseholderQr().solve(b);
    a3 = x(0);
    a4 = x(1);
}

QuarticPolynomial::~QuarticPolynomial()
{
    cout<<"deleted!"<<endl;
}

double QuarticPolynomial::calc_point(double t)
{
    double xt;
    xt = a0 + a1*t + a2 * pow(t,2)  + a3 * pow(t,3) + a4 * pow(t,4);
    return xt;
}

double QuarticPolynomial::calc_first_derivative(double t)
{
    double xt;
    xt = a1+ 2*a2 * t  + 3*a3 * pow(t,2) + 4*a4 * pow(t,3);
    return xt;
}

double QuarticPolynomial::calc_second_derivative(double t)
{
    double xt;
    xt = 2*a2  + 6*a3 * t + 12*a4 * pow(t,2);
    return xt;
}

double QuarticPolynomial::calc_third_derivative(double t)
{
    double xt;
    xt =  6*a3  + 24*a4 * t;
    return xt;
}