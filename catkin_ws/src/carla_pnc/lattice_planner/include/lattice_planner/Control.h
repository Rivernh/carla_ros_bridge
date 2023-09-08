#ifndef CONTROL_H  // 防止头文件被重复包含
#define CONTROL_H
#include <iostream>
#include<eigen3/Eigen/Dense>
#include<vector>
#include<cmath>
#include<ctime>
#include <chrono>
#include<lattice_planner/Spline.h>

using namespace std;
using namespace Eigen;

struct Para
{
    /* data */
    // vehicle
    double L = 2.405;
    double max_acc = 1.0;
    double min_acc = -1.0;
    double max_steer = 1.0;

    // RWFB
    double k_k = 1.235;
    double k_theta = 0.456;
    double k_e = 0.11;

    // debug
    double curvature_factor = 1.0;
};

class IncreasePID
{
private:
    /* data */
    double kp,ki,kd;
    double last,lastlast;
public:
    IncreasePID(double kp=0.3, double ki=0.002,double kd = 0.5);
    ~IncreasePID();
    double run(double target,double now);
};

struct CarlaControl
{
    /* data */
    double throttle;
    double steer;
    double brake;
};

class State
{
private:
    /* data */
public:
    string frame_id;
    double time_stamp;
    double x,y,z,theta,k,s,v,a,t;
    double *velocity;
    double *acceleration;
    State(string frame_id,double time_stamp,point pt);
    ~State();
    State word_to_local_2D(State state0,string local_frame_id);
};

State getActorState(string frame_id,double time_stamp,point pt);

class RwfbControl
{
private:
    /* data */
    Para para;
    double L;
    double max_steer;
    double dt;
    double curvature_facor;
    double throttle;
    double brake;
    double acc;
    double steer;
    double k_k,k_theta,k_e;
    IncreasePID PIDer;

public:
    RwfbControl(double frequence);
    ~RwfbControl();
    CarlaControl pid(State c_s,State t_s);
    double rwpf(State c_s,State t_s);
    CarlaControl run(trajectory traj,int index,State state0,point pt);
};


#endif


