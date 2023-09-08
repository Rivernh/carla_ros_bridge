#include<lattice_planner/Planner.h>

LatticePlanner::LatticePlanner(Config conf)
{
    config.max_speed = conf.max_speed;
}

LatticePlanner::~LatticePlanner()
{
}

FrenetPath *LatticePlanner::calc_frenet_paths(double target_speed,double  c_speed, frenet_point fre_pt)
{
    FrenetPath *frenet_paths;
    int index = 0;
    for(int i = 0;i < int((config.MAX_ROAD_WIDTH - config.MIN_ROAD_WIDTH) / config.d_road_w);i++)
    {
        double di = config.MIN_ROAD_WIDTH + i * config.d_road_w;
        for(int j = 0; j < int((config.max_t - config.min_t) / config.dt);j++)
        {
            double Ti  = config.min_t + j * config.dt;
            int nx;
            FrenetPath fp;
            QuinticPolynomial lat_qp = QuinticPolynomial(fre_pt.d,fre_pt.d_d,fre_pt.d_dd,di,0.0,0.0,Ti);
            for(nx = 0;nx * config.dt < Ti;nx ++)
            {
                fp.t[nx] = nx * config.dt;
                fp.d[nx] = lat_qp.calc_point(fp.t[nx]);
                fp.d_d[nx] = lat_qp.calc_first_derivative(fp.t[nx]);
                fp.d_dd[nx] = lat_qp.calc_second_derivative(fp.t[nx]);
                fp.d_ddd[nx] = lat_qp.calc_third_derivative(fp.t[nx]);
            }
            for (int k = 0; k < 2 * config.n_s_sample; k++)
            {
                double tv = target_speed - config.n_s_sample * config.d_speed + k * config.d_speed;
                FrenetPath tfp;
                QuarticPolynomial lon_qp = QuarticPolynomial(fre_pt.s,fre_pt.s_d,0.0,tv,0.0,Ti);
                for(int l = 0;l < nx;l ++)
                {
                    tfp.t[l] =  fp.t[l];
                    tfp.d[l] = fp.d[l];
                    tfp.d_d[l] = fp.d_d[l];
                    tfp.d_dd[l] = fp.d_dd[l];
                    tfp.d_ddd[l] = fp.d_ddd[l];
                    tfp.s[l] = lon_qp.calc_point(fp.t[l]);
                    tfp.s_d[l] = lon_qp.calc_first_derivative(fp.t[l]);
                    tfp.s_dd[l] = lon_qp.calc_second_derivative(fp.t[l]);
                    tfp.s_ddd[l] = lon_qp.calc_third_derivative(fp.t[l]);
                }
                double Jp = 0,Js = 0;
                for(int l = 0;l < nx;l++)
                {
                    Jp += pow(tfp.d_ddd[l],2);
                    Js  += pow(tfp.s_ddd[l],2);
                }
                double ds = pow(target_speed - tfp.s[nx-1],2);
                tfp.cd = config.K_J * Jp + config.K_T * Ti + config.K_D * pow(tfp.d[nx-1],2);
                tfp.cv = config.K_J * Js  + config.K_T * Ti + config.K_D * ds;
                tfp.cf = config.K_LAT * tfp.cd + config.K_LON * tfp.cv;
                // 重写符号重载
                frenet_paths[index ++] = tfp;
            }
        }
    }
    return frenet_paths;
    
}

FrenetPath *LatticePlanner::calc_global_paths(FrenetPath *fplist,Spline2D csp)
{
    int nx =  int(sizeof(fplist)/sizeof(fplist[0]));
    for(int i = 0; i < nx; i++)
    {
        int nxx =  int(sizeof(fplist[i].s)/sizeof(fplist[i].s[0]));
        for(int j = 0; j < nxx;j ++)
        {
            double rx = csp.calc_pos(fplist[i].s[j]).x;
            double ry = csp.calc_pos(fplist[i].s[j]).y;
            if(rx == 0.0)
                break;
            double rtheta = csp.calc_yaw(fplist[i].s[j]);
            double rkappa = csp.calc_curv(fplist[i].s[j]);
            double rdkappa = csp.calc_d_curv(fplist[i].s[j]);

            frenet_point fre_p;
            fre_p.s = fplist[i].s[j];
            fre_p.s_d = fplist[i].s_d[j];
            fre_p.s_dd = fplist[i].s_dd[j];
            fre_p.d = fplist[i].d[j];
            fre_p.d_d = fplist[i].d_d[j];
            fre_p.d_dd = fplist[i].d_dd[j];

            point car_p = frenet_to_cartesian3D(fplist[i].s[j],rx,ry,rtheta,rkappa,rdkappa,fre_p);

            fplist[i].x[j] = car_p.x;
            fplist[i].y[j] = car_p.y;
            fplist[i].v[j] = car_p.v;
            fplist[i].a[j] = car_p.a;
            fplist[i].yaw[j] = car_p.theta;
            fplist[i].c[j] = car_p.kappa;
        }
    }
    return fplist;  
}

bool LatticePlanner::check_collision(FrenetPath fp,Obstacle *ob)
{
    int nx = int(sizeof(ob)/sizeof(ob[0]));
    if(nx == 0)
        return true;

}

FrenetPath *LatticePlanner::check_paths(FrenetPath *fplist,Obstacle *ob)
{
    FrenetPath *fplist_return;
    int nx =  int(sizeof(fplist)/sizeof(fplist[0]));
    double ok_ind[nx];
    int index = 0;
    int flag = 0;
    for(int i = 0; i < nx; i ++)
    {
        ok_ind[i] = 0;
        int nnx = int(sizeof(fplist[i].s_d)/sizeof(fplist[i].s_d[0]));
        flag = 0;
        for(int j = 0; j < nnx; j++)
        {
            if(fplist[i].s_d[j] > config.max_speed)
                flag = 1;
            // 其他限制
        }
        if(!check_collision(fplist[i],ob))
            flag = 1;
        if(!flag)
            ok_ind[i] = 1;
    }
    for(int i = 0; i < nx;i++)
    {
        // 这个赋值需要符号重载
        //TODO
        if(ok_ind[i])
            fplist_return[index++] = fplist[i];
    }
    return fplist_return;
}

FrenetPath LatticePlanner::frenet_optimal_planning(double target_speed,double  c_speed,Spline2D csp, frenet_point fre_pt,Obstacle *ob)
{
    FrenetPath *fplist = calc_frenet_paths(target_speed,c_speed,fre_pt);
    fplist = calc_global_paths(fplist,csp);
    fplist = check_paths(fplist,ob);
    double min_cost = float(10e6);
    FrenetPath *best_path;
    int nx = int(sizeof(fplist)/sizeof(fplist[0]));
    for(int i = 0; i < nx; i++)
    {
        if(min_cost >=  fplist[i].cf)
        {
            min_cost = fplist[i].cf;
            best_path = &fplist[i];
        }
    }
    return *best_path;
}



