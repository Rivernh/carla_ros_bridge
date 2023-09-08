#ifndef PLANNER_H  // 防止头文件被重复包含
#define PLANNER_H
#include <iostream>
#include<eigen3/Eigen/Dense>
#include<vector>
#include<cmath>
#include<lattice_planner/Spline.h>

using namespace std;

struct FrenetPath
{
    /* data */
    double *t;
    double *d;
    double *d_d;
    double *d_dd;
    double *d_ddd;
    double *s;
    double *s_d;
    double *s_dd;
    double *s_ddd;
    double cd;
    double cv;
    double cf;

    double *x;
    double *y;
    double *yaw;
    double *v;
    double *a;
    double *c;
};

struct Config
{
    /* data */
    double max_speed;
    double max_acc;
    double max_speed;
    double max_acc;
    double max_curvature;
    double MIN_ROAD_WIDTH;
    double MAX_ROAD_WIDTH;
    double d_road_w;
   double dt;
    double max_t;
    double min_t;
    double n_s_sample;
    double d_speed;
    //TARGET_SPEED: 16.67 #60 / 3.6  # target speed [m/s]
    double ROBOT_RADIUS;

    //Parameters for constant distance and constant time law.
    double D0;
    double tau;

    double K_J;
    double K_T;
    double K_D;
    double K_LAT;
    double K_LON;

    // car
    double L;
    double W;
};

struct Obstacle
{
    /* data */
    point pos;
    double width,length;
};

class LatticePlanner
{
private:
    /* data */
    Config config;
public:
    LatticePlanner(Config conf);
    ~LatticePlanner();
    FrenetPath *calc_frenet_paths(double target_speed,double  c_speed, frenet_point fre_pt);
    FrenetPath *calc_global_paths(FrenetPath *fplist,Spline2D csp);
    bool check_collision(FrenetPath fp,Obstacle *ob);
    FrenetPath *check_paths(FrenetPath *fplist,Obstacle *ob);
    FrenetPath frenet_optimal_planning(double target_speed,double  c_speed,Spline2D csp, frenet_point fre_pt,Obstacle *ob);
};


#endif