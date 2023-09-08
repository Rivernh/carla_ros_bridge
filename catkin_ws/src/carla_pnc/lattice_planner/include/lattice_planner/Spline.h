#ifndef SPLINE_H  // 防止头文件被重复包含
#define SPLINE_H
#include <iostream>
#include<eigen3/Eigen/Dense>
#include<vector>
#include<cmath>

using namespace std;
using namespace Eigen;

#define MAX_LEN 100

struct frenet_point
{
    /* data */
    double s,d;
    double s_d,d_d;
    double s_dd,d_dd;
};

struct point
{
    /* data */
    double x,y,z=0;
    double v,theta;
    double a,kappa;
    double t,s;
};

struct curve_cartesian
{
    /* data */
    double *rx,*ry,*ryaw,*rk,*s;
};

struct trajectory
{
    /* data */
    double *x,*y,*a,*v,*yaw,*c;
    double time;
};


struct course1D
{
    /* data */
};

struct course2D
{
    /* data */
    double *rx,*ry,*ryaw,*rk,*rdk;
    curve_cartesian csp;
    double *s;
};

class Spline
{
private:
    /* data */
    double *x,*y;
    int nx;
    double a[MAX_LEN];
    Eigen::MatrixXd c;
    double b[MAX_LEN],d[MAX_LEN],w[MAX_LEN];

public:
    Spline();
    Spline(double *x,double *y);
    ~Spline();
    Eigen::MatrixXd __calc_A(double *h);
    Eigen::MatrixXd __calc_B(double *h);
    double calc(double t);
    double calcd(double t);
    double calcdd(double t);
    double calcddd(double t);
    int __search_index(double x);
};

class Spline2D
{
private:
    /* data */
    int nx;
    Spline sx,sy;

public:
    double *s;
    Spline2D(double *x, double *y);
    ~Spline2D();
    point calc_pos(double s);
    double calc_curv(double s);
    double calc_d_curv(double s);
    double calc_yaw(double s);
    curve_cartesian calc_spline_course(double *x, double *y, double ds = 0.1);
};

class QuarticPolynomial
{
private:
    /* data */
    double a0,a1,a2,a3,a4;

public:
    QuarticPolynomial(double xs,double vs,double as,double ve, double ae, double time);
    ~QuarticPolynomial();
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
};

class QuinticPolynomial
{
private:
    /* data */
    double a0,a1,a2,a3,a4,a5;

public:
    QuinticPolynomial(double xs,double vs,double as,double xe, double ve, double ae, double time);
    ~QuinticPolynomial();
    double calc_point(double t);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
};

class FreNetMap
{
private:
    /* data */
public:
    course2D course;
    FreNetMap(course2D input_course);
    ~FreNetMap();
};


double NormalizeAngle(double angle);
frenet_point cartesian_to_frenet1D(double rs,double rx,double ry,double rtheta,point p_car);
point frenet_to_cartesian1D(double rs,double rx,double ry,double rtheta,frenet_point p_fre);
frenet_point cartesian_to_frenet2D(double rs,double rx,double ry,double rtheta,double rkappa,point p_car);
point frenet_to_cartesian2D(double rs,double rx,double ry,double rtheta,double rkappa,frenet_point p_fre);
frenet_point cartesian_to_frenet3D(double rs,double rx,double ry,double rtheta,double rkappa,double r_dkappa,point p_car);
point frenet_to_cartesian3D(double rs,double rx,double ry,double rtheta,double rkappa,double rdkappa,frenet_point p_fre);
course1D generate_target_course(double *x,double *y);
course2D generate_target_course2(double *x,double *y);

#endif