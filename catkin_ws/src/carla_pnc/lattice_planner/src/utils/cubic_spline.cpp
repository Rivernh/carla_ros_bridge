#include<lattice_planner/Spline.h>

Spline::Spline(double *x,double *y)
{
    this->x = x;
    this->y = y;
    this->nx = int(sizeof(x) / sizeof(x[0]));
    double h[this->nx];
    for(int i=0;i<this->nx-1;i++)
    {
        h[i] = this->x[i+1] - this->x[i]; 
    }
    for (int i =0;i<nx;i++)
        a[i] = y[i];

    MatrixXd A = __calc_A(h);
    MatrixXd B = __calc_B(h);

    c = A.colPivHouseholderQr().solve(B);

    for(int i=0;i<nx-1;i++)
    {
        d[i] = (c(i+1) - c(i)) / (3.0 * h[i]);
        b[i] = (a[i+1] - a[i]) /h[i] -h[i] * (c(i+1) + 2.0 * c(i)) / 3.0;
    }
}

Spline::Spline()
{
    cout<<"create successful"<<endl;
}

Spline::~Spline()
{
    cout<<"delete!"<<endl;
}

Eigen::MatrixXd Spline::__calc_A(double *h)
{
    MatrixXd A(nx,nx);
    A(0,0)=1.0;
    for(int i =0;i<nx-1;i++)
    {
        if(i != (nx-2))
        {
            A(i+1,i+1) = 2.0*(h[i] + h[i + 1]);
        }
        A(i+1,i) = h[i];
        A(i, i+1) = h[i];
    }
    A(0,1) = 0.0;
    A(nx-1,nx-2) = 0.0;
    A(nx-1,nx-1) = 1.0;
    return A;
}

Eigen::MatrixXd Spline::__calc_B(double *h)
{
    MatrixXd B(nx,1);
    for(int i =0;i<nx-2;i++)
    {
        B(i+1)=3.0 * (a[i+2] - a[i+1])/ h[i+1] - 3.0*(a[i+1] - a[i])/h[i];
    }
    B(0) = 0;
    B(nx-1) = 0;
    return B;
}

double Spline::calc(double t)
{
    int i;
    double result,dx;
    if((t > x[0]) && (t < x[nx-1]))
    {
        i = __search_index(t);
        dx = t - x[i];
        result = a[i] + b[i] *dx + c(i) *pow(dx,2) + d[i] * pow(dx,3);
        return result;
    }
    else 
    {
        cout<<"Spline calc input error!"<<endl;
        return 0;
    }
}

double Spline::calcd(double t)
{
    int i;
    double result,dx;
    if((t > x[0]) && (t < x[nx-1]))
    {
        i = __search_index(t);
        dx = t - x[i];
        result = b[i]  + 2 * c(i) * dx +  3 * d[i] * pow(dx,2);
        return result;
    }
    else 
    {
        cout<<"Spline calc input error!"<<endl;
        return 0;
    }
}

double Spline::calcdd(double t)
{
    int i;
    double result,dx;
    if((t > x[0]) && (t < x[nx-1]))
    {
        i = __search_index(t);
        dx = t - x[i];
        result = 2 * c(i) +  6 * d[i] * dx;
        return result;
    }
    else 
    {
        cout<<"Spline calc input error!"<<endl;
        return 0;
    }
}

double Spline::calcddd(double t)
{
    int i;
    double result,dx;
    if((t > x[0]) && (t < x[nx-1]))
    {
        i = __search_index(t);
        dx = t - x[i];
        result =  6 * d[i];
        return result;
    }
    else 
    {
        cout<<"Spline calc input error!"<<endl;
        return 0;
    }
}

int Spline::__search_index(double x)
{
    int i;
    for(i = 0;i <nx;i++)
    {
        if(this->x[i] > x)
            return i-1;
    }
    return nx-1;
}

Spline2D::Spline2D(double *x, double *y)
{
    nx =  int(sizeof(x) / sizeof(x[0]));
    double ds[nx-1],s[nx];
    s[0] = 0;
    for(int i = 0;i<nx-1;i++)
    {
        ds[i] = (x[i+1] - x[i]) * (x[i+1] - x[i]) + (y[i+1] -y[i]) * (y[i+1] -y[i]);
        s[i + 1] = s[i] + ds[i];
    }
    this->s = s;
    this->sx = Spline(this->s,x);
    this->sy = Spline(this->s,y);
}

Spline2D::~Spline2D()
{
    cout<<"deleted!"<<endl;
}

point Spline2D::calc_pos(double s)
{
    point pos;
    pos.x = sx.calc(s);
    pos.y = sy.calc(s);
    return pos;
}

double Spline2D::calc_curv(double s)
{
    double dx,ddx,dy,ddy,k;
    dx =sx.calcd(s);
    ddx = sx.calcdd(s);
    dy = sy.calcd(s);
    ddy = sy.calcdd(s);
    k = (ddy*dx -ddx*dy) / pow((dx*dx + dy*dy),3/2);
    return k;
}

double Spline2D::calc_d_curv(double s)
{
    double dx,ddx,dddx,dy,ddy,dddy,a,b,c,d;
    dx =sx.calcd(s);
    ddx = sx.calcdd(s);
    dddx = sx.calcddd(s);
    dy = sy.calcd(s);
    ddy = sy.calcdd(s);
    dddy = sy.calcddd(s);

    a = dx*ddy-dy*ddx;
    b =dx*dddy-dy*dddx;
    c = dx*ddx + dy*ddy;
    d =dx*dx + dy*dy;
    return (b*d -3.0* a* c) / (d*d*d);
}

double Spline2D::calc_yaw(double s)
{
    double dx,dy,yaw;
    dx = sx.calcd(s);
    dy = sy.calcd(s);
    yaw = atan2(dy,dx);
    return yaw;
}

curve_cartesian Spline2D::calc_spline_course(double *x, double *y, double ds)
{
    Spline2D sp(x,y);
    curve_cartesian result;
    point temp;
    int cnt;
    for(cnt = 0; cnt * ds <sp.s[-1];cnt++)
    {
        result.s[cnt] = cnt* ds;
    }
    for(int i =0;i<cnt;i++)
    {
        temp = sp.calc_pos(i * ds);
        result.rx[i] = temp.x;
        result.ry[i] = temp.y;
        result.ryaw[i] = sp.calc_yaw(i * ds);
        result.rk[i] = sp.calc_curv(i *ds);
    }
    return result;
}




