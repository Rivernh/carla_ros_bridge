#include<lattice_planner/Spline.h>

double NormalizeAngle(double angle)
{
    double a = fmod(angle + M_PI,2 * M_PI);
    if(a < 0.0 )
        a += (2 * M_PI);
    return a - M_PI;
}

frenet_point cartesian_to_frenet1D(double rs,double rx,double ry,double rtheta,point p_car)
{
    double dx,dy,cos_thata_r,sin_theta_r,cross_rd_nd;
    double x = p_car.x,y = p_car.y;
    frenet_point  result;
    dx = x -rx;
    dy = y - ry;
    cos_thata_r = cos(rtheta);
    sin_theta_r = sin(rtheta);
    cross_rd_nd = cos_thata_r * dy - sin_theta_r * dx;
    result.d = copysign(sqrt(dx *dx + dy *dy),cross_rd_nd);
    result.s = rs;
    return result;
}

point frenet_to_cartesian1D(double rs,double rx,double ry,double rtheta,frenet_point p_fre)
{
    double cos_theta_r,sin_theta_r;
    double s = p_fre.s,d = p_fre.d;
    point result;
    if(fabs(rs - s) >= 1.0e-6)
        cout<<"The reference point s and s_condition[0] don't match"<<endl;
    cos_theta_r = cos(rtheta);
    sin_theta_r = sin(rtheta);
    result.x = rx -sin_theta_r *d;
    result.y = ry + cos_theta_r *d;
    return result;
}

frenet_point cartesian_to_frenet2D(double rs,double rx,double ry,double rtheta,double rkappa,point p_car)
{
    double dx,dy,cos_thata_r,sin_theta_r,cross_rd_nd;
    double delta_theta,tan_delta_theta,cos_delta_theta,one_minus_kappa_r_d;
    double x = p_car.x,y =p_car.y,v = p_car.v,theta = p_car.theta;
    frenet_point  result;
    dx = x -rx;
    dy = y - ry;
    cos_thata_r = cos(rtheta);
    sin_theta_r = sin(rtheta);
    cross_rd_nd = cos_thata_r * dy - sin_theta_r * dx;
    result.d = copysign(sqrt(dx *dx + dy *dy),cross_rd_nd);

    delta_theta = theta - rtheta;
    tan_delta_theta = tan(delta_theta);
    cos_delta_theta = cos(delta_theta);

    one_minus_kappa_r_d = 1- rkappa * result.d;
    result.d_d = one_minus_kappa_r_d * tan_delta_theta;

    result.s = rs;
    result.s_d = v * cos_delta_theta / one_minus_kappa_r_d;
    return result;
}

point frenet_to_cartesian2D(double rs,double rx,double ry,double rtheta,double rkappa,frenet_point p_fre)
{
    double cos_theta_r,sin_theta_r;
    double delta_theta,tan_delta_theta,cos_delta_theta,one_minus_kappa_r_d,d_dot;
    double s = p_fre.s,s_d = p_fre.s_d,d = p_fre.d,d_d = p_fre.d_d;
    point result;
    if(fabs(rs - s) >= 1.0e-6)
        cout<<"The reference point s and s_condition[0] don't match"<<endl;

    cos_theta_r = cos(rtheta);
    sin_theta_r = sin(rtheta);

    one_minus_kappa_r_d = 1- rkappa * d;
    tan_delta_theta = d_d / one_minus_kappa_r_d;
    delta_theta = atan2(d_d,one_minus_kappa_r_d);
    cos_delta_theta = cos(delta_theta);

    result.theta = NormalizeAngle(delta_theta + rtheta);
    
    d_dot = d_d * s_d;
    
    result.x = rx -sin_theta_r *d;
    result.y = ry + cos_theta_r *d;
    result.v = sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_d * s_d + d_dot * d_dot);
    return result;
}

frenet_point cartesian_to_frenet3D(double rs,double rx,double ry,double rtheta,double rkappa,double r_dkappa,point p_car)
{
    double dx,dy,cos_thata_r,sin_theta_r,cross_rd_nd;
    double delta_theta,tan_delta_theta,cos_delta_theta,one_minus_kappa_r_d,kappa_r_d_prime,delta_theta_prime;
    double x = p_car.x,y =p_car.y,v = p_car.v,theta = p_car.theta,a = p_car.a,kappa = p_car.kappa;
    frenet_point  result;
    dx = x -rx;
    dy = y - ry;
    cos_thata_r = cos(rtheta);
    sin_theta_r = sin(rtheta);
    cross_rd_nd = cos_thata_r * dy - sin_theta_r * dx;
    result.d = copysign(sqrt(dx *dx + dy *dy),cross_rd_nd);

    delta_theta = theta - rtheta;
    tan_delta_theta = tan(delta_theta);
    cos_delta_theta = cos(delta_theta);

    one_minus_kappa_r_d = 1- rkappa * result.d;
    result.d_d = one_minus_kappa_r_d * tan_delta_theta;

    kappa_r_d_prime = r_dkappa * result.d + r_dkappa * result.d_d;
    result.d_dd = (-kappa_r_d_prime * tan_delta_theta + 
    one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
    (kappa * one_minus_kappa_r_d/ cos_delta_theta - rkappa));

    result.s = rs;
    result.s_d = v * cos_delta_theta / one_minus_kappa_r_d;
    delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa -rkappa;
    result.s_dd = ((a * cos_delta_theta - result.s_d * result.s_d * 
    (result.d_d * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d);
    return result;
}

point frenet_to_cartesian3D(double rs,double rx,double ry,double rtheta,double rkappa,double rdkappa,frenet_point p_fre)
{
    double cos_theta_r,sin_theta_r;
    double delta_theta,tan_delta_theta,cos_delta_theta,one_minus_kappa_r_d,d_dot,kappa_r_d_prime,delta_theta_prime;
    double s = p_fre.s,s_d = p_fre.s_d,s_dd = p_fre.s_dd,d = p_fre.d,d_d = p_fre.d_d,d_dd = p_fre.d_dd;
    point result;
    if(fabs(rs - s) >= 1.0e-6)
        cout<<"The reference point s and s_condition[0] don't match"<<endl;

    cos_theta_r = cos(rtheta);
    sin_theta_r = sin(rtheta);

    one_minus_kappa_r_d = 1- rkappa * d;
    tan_delta_theta = d_d / one_minus_kappa_r_d;
    delta_theta = atan2(d_d,one_minus_kappa_r_d);
    cos_delta_theta = cos(delta_theta);

    result.theta = NormalizeAngle(delta_theta + rtheta);
    kappa_r_d_prime =  rdkappa * d + rkappa *d_d;

    result.kappa = ((((d_dd + kappa_r_d_prime * tan_delta_theta) *
     cos_delta_theta * cos_delta_theta) / (one_minus_kappa_r_d) +
      rkappa) * cos_delta_theta / (one_minus_kappa_r_d));
    
    d_dot = d_d * s_d;
    
    result.x = rx -sin_theta_r *d;
    result.y = ry + cos_theta_r *d;
    result.v = sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_d * s_d + d_dot * d_dot);

    delta_theta_prime =  one_minus_kappa_r_d / cos_delta_theta * result.kappa * rkappa;
    result.a = (s_dd * one_minus_kappa_r_d / cos_delta_theta + s_d * s_d / cos_delta_theta *
     (d_d * delta_theta_prime - kappa_r_d_prime));
    return result;
}

