#include<lattice_planner/Spline.h>

FreNetMap::FreNetMap(course2D input_course)
{
    course.rx = input_course.rx;
    course.ry = input_course.ry;
    course.ryaw = input_course.ryaw;
    course.rk = input_course.rk;
    course.rdk = input_course.rdk;
    course.csp = input_course.csp;
    course.s = input_course.s;
}

FreNetMap::~FreNetMap()
{
}

FreNetMap generate_frenet_map(double *x,double *y)
{

}

course1D generate_target_course(double *x,double *y)
{
    Spline2D csp = Spline2D(x,y);
    double *s;
    course1D result;

}

course2D generate_target_course2(double *x,double *y)
{
    int nx =  int(sizeof(x) / sizeof(x[0]));
    course2D returnValue;
    returnValue.csp = Spline2D(x,y);
    for(int i = 0;i * 0.1 < returnValue.csp[nx-1];i++ )
    {
        returnValue.s[i] = i *0.1;
        returnValue.rx[i] = returnValue.csp.calc_pos(i*0.1).x;
        returnValue.ry[i] = returnValue.csp.calc_pos(i*0.1).y;
        returnValue.ryaw[i] = csreturnValue.cspp.calc_yaw(i*0.1);
        returnValue.rk[i] = returnValue.csp.calc_curv(i*0.1);
        returnValue.rdk[i] = returnValue.csp.calc_d_curv(i*0.1);
    }
    return returnValue;
}





