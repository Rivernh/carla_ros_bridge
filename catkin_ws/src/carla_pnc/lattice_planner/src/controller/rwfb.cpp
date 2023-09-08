#include<lattice_planner/Control.h>

/*  PID  */
IncreasePID::IncreasePID(double kp, double ki,double kd)
{
    this->kp = kp;
    this->ki   = ki;
    this->kd = kd;
    last = 0.0;
    lastlast = 0.0;
}

IncreasePID::~IncreasePID()
{
}

double IncreasePID::run(double target,double now)
{
    double error  =  target - now;
    double output = kp * (error - last) + ki * error + kd * (error - 2 * last + lastlast);
    lastlast = last;
    last = error;
    return output;
}

/* State*/
State::State(string frame_id,double time_stamp,point pt)
{
    this->frame_id = frame_id;
    this->time_stamp = time_stamp;
    x = pt.x;
    y = pt.y;
    z = pt.z;
    theta = pt.theta;
    k = pt.kappa;
    s = pt.s;
    v = pt.v;
    a = pt.a;
    t = pt.t;
}

State::~State()
{
}

State State::word_to_local_2D(State state0,string local_frame_id)
{
    if(frame_id != state0.frame_id)
        cout<<'carla_utils/augment/State: Wrong1'<<endl;
    double delta_theta = NormalizeAngle(theta - state0.theta);
    double x_local =  (x - state0.x) * cos(state0.theta) + (y - state0.y) * sin(state0.theta);
    double y_local = -(x - state0.x) * sin(state0.theta) + (y - state0.y) * cos(state0.theta);
    point pt;
    pt.x = x_local;
    pt.y = y_local;
    pt.theta = delta_theta;
    pt.z = z;
    pt.kappa = k;
    pt.s = s;
    pt.v = v;
    pt.t= t;
    State local_state = State(local_frame_id,time_stamp,pt);
    
    return local_state;
}

/* function*/
State getActorState(string frame_id,double time_stamp,point pt)
{
    State return_value = State(frame_id,time_stamp,pt);
}

/* RWFB */
RwfbControl::RwfbControl(double frequence)
{
    L = para.L;
    max_steer = para.max_steer;
    dt = 1.0 / frequence;
    curvature_facor = para.curvature_factor;
    throttle = 0.0;
    brake = 0.0;
    steer = 0.0;
    PIDer = IncreasePID(2.00,0.05,0.5);
    k_k = para.k_k;
    k_theta = para.k_theta;
    k_e = para.k_e;
}

RwfbControl::~RwfbControl()
{
}

CarlaControl RwfbControl::run(trajectory traj,int index,State state0, point pt)
{
    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    // 将时间戳转换为毫秒数
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto value = now_ms.time_since_epoch().count();
    double t_stamp = value * 0.001;
    State current_state = getActorState(string("odom"),t_stamp,pt);
    current_state = current_state.word_to_local_2D(state0,string("base_link"));

    point target_pt;
    target_pt.x = traj.x[index];
    target_pt.y = traj.y[index];
    target_pt.a = traj.a[index];
    target_pt.kappa = traj.c[index];
    target_pt.v = traj.v[index];
    target_pt.theta = traj.yaw[index];
    target_pt.t  = traj.time;
    State target_state = State(string("base_link"),t_stamp,target_pt);

    double s = rwpf(current_state,target_state);
    CarlaControl cmd = pid(current_state,target_state);
    
    target_state.k = target_pt.kappa * curvature_facor;

    double ks = 0.5;
    if(fabs(s) > 0.5)
        ks = 0.9;

    s = s * ks;
    cmd.steer = s;
    steer = s;
    return cmd;
}

CarlaControl RwfbControl::pid(State c_s,State t_s)
{
    double v_current = c_s.v;
    double v_target = t_s.v;
    double t,b;
    CarlaControl cmd;
    acc += PIDer.run(v_target,v_current);
    if(acc >= 0)
    {
        if(acc > 1)
            acc = 1;
        t = acc;
        b = 0.0;
    }
    else
    {
        if(acc < -1)
            acc = -1;
        b = acc;
        t = 0.0;
    }
    cmd.throttle = t;
    cmd.brake = b;
    return cmd;
}

double RwfbControl::rwpf(State c_s,State t_s)
{
    double xr = t_s.x;
    double yr = t_s.y;
    double thetar = t_s.theta;
    double vr = t_s.v;
    double kr = t_s.k;

    double dx = c_s.x - xr;
    double dy = c_s.y - yr;
    double tx = cos(thetar);
    double ty = sin(thetar);
    double e = dx*ty - dy*tx;
    double theta_e =NormalizeAngle(c_s.theta - thetar);

    double alpha = 1.8;
    double w1 =k_k * vr * kr * cos(theta_e);
    double w2 = -k_theta * fabs(vr) * theta_e;
    double w3 =(k_e * vr * exp(-theta_e *theta_e / alpha)) * e;
    double w = (w1 +w2 +w3) * 0.8;

    double s;
    if(c_s.v < 0.02)
        s = 0;
    else
        s = atan2(w * L,c_s.v) * 2 / M_PI * max_steer;

    return s;
}

