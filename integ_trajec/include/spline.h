#ifndef SPLINE_H
#define SPLINE_H

// explanation at the end of the library

#include <cstdio>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <iostream>



namespace tk
{

class spline
{
public:

    // boundary condition type for the spline end-points
    enum bd_type {
        first_deriv = 1,
        second_deriv = 2
    };

    std::vector<double> m_x,m_y;            // x,y coordinates of points
    // interpolation parameters
    // f(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3
    // where a_i = y_i, or else it won't go through grid points
    std::vector<double> m_b,m_c,m_d;        // spline coefficients
    double m_c0;                            // for left extrapolation
    bd_type m_left, m_right;
    double  m_left_value, m_right_value;
    void set_coeffs_from_b();               // calculate c_i, d_i from b_i
    size_t find_closest(double x) const;    // closest idx so that m_x[idx]<=x

    // default constructor: set boundary condition to be zero curvature
    spline():
        m_left(second_deriv), m_right(second_deriv),
        m_left_value(0.0), m_right_value(0.0)
    {
        ;
    }

    spline(const std::vector<double>& X, const std::vector<double>& Y,
           bd_type left  = first_deriv, double left_value  = 0.0,
           bd_type right = first_deriv, double right_value = 0.0):
        m_left(left), m_right(right),
        m_left_value(left_value), m_right_value(right_value)
    {
        this->set_points(X,Y);
    }


    // modify boundary conditions: if called it must be before set_points()
    void set_boundary(bd_type left, double left_value,
                      bd_type right, double right_value);

    // set all data points
    void set_points(const std::vector<double>& x,
                    const std::vector<double>& y);

    // evaluates the spline at point x
    double operator() (double x) const;

    double deriv(int order, double x) const;

};

// 2D spline interpolation
class Ispline
{
public:
    std::vector<double> x, y;        // x,y,z coordinates of points and date
    double amax = 0, vmax = 0;
    std::vector<double> t;
    tk::spline sx = spline(t,x);
    tk::spline sy = spline(t,y);


    Ispline():
        x(),y()
        {}

    Ispline(const std::vector<double>& X, const std::vector<double>& Y,
            double VMAX, double AMAX):
        x(X), y(Y), t(Itemps(X, Y, VMAX, AMAX))
        {
            sx = spline(t, X);
            sy = spline(t, Y);

            amax = AMAX;
            vmax = VMAX;
            x = X;
            y = Y;

            this->I_set_points();
        }

    void I_set_coeffs_from_b()    // calculate c_i, d_i from b_i
    {
        sx.set_coeffs_from_b();
        sy.set_coeffs_from_b();
    }

    const std::vector<size_t> I_find_closest(const std::vector<double> pos)
    {
        const std::vector<size_t> result =
        {
        sx.find_closest(pos[0]),
        sy.find_closest(pos[1])
        };
        return result;
    }

    // modify boundary conditions: if called it must be before F_set_points()
    void I_set_boundary()
    {
        sx.set_boundary(sx.m_left, sx.m_left_value, sx.m_right, sx.m_right_value);
        sy.set_boundary(sy.m_left, sy.m_left_value, sy.m_right, sy.m_right_value);
    }

    // set all data points
    void I_set_points()
    {
        sx.set_points(t, x);
        sy.set_points(t, y);
    }

    // evaluates the spline at a vector pos
    const std::vector<double> Ioperator(spline sx, spline sy, spline sz, std::vector<double> pos)
    {
        std::vector<double> result =
        {
            sx.operator()(pos[0]),
            sy.operator()(pos[1]),
            sz.operator()(pos[2])
        };
        return result;
    }

    const std::vector<double> Ideriv(int order, double t) const
    {
        double dx = sx.deriv(order, t);
        double dy = sy.deriv(order, t);
        return std::vector<double> {dx, dy};
    }


std::vector<double> Itemps(std::vector<double> x,
                          std::vector<double> y,
                          double VMAX = 0, double AMAX = 0)
{
    std::vector<double> t = Iset_time(x, y, VMAX, AMAX);
    tk::spline sx, sy;
    sx = spline(t, x);
    sy = spline(t, y);
    int n = t.size();
    double dt = VMAX/((std::abs(x[1])+std::abs(y[1]))*2);

    double aix = std::abs(sx.deriv(2,t[0]+0.01));
    double aiy = std::abs(sy.deriv(2,t[0]+0.01));


    while (aix > AMAX || aiy > AMAX)
    {
        for (int j = 1; j < n; ++j)
            t[j] += dt;
        sx = spline(t, x);
        sy = spline(t, y);
        aix = std::abs(sx.deriv(2,t[0]+0.01));
        aiy = std::abs(sy.deriv(2,t[0]+0.01));
    }


    for (int i = 0; i < n; i++)
    {
        double vix = set_v(sx, t[i], t[i+1], i);
        double viy = set_v(sy, t[i], t[i+1], i);
        double aix = std::max(std::abs(sx.deriv(2,t[i]+0.01)),std::abs(sx.deriv(2,t[i+1]-0.01)));
        double aiy = std::max(std::abs(sy.deriv(2,t[i]+0.01)),std::abs(sy.deriv(2,t[i+1]-0.01)));



        while (vix > VMAX || viy > VMAX || aix > AMAX || aiy > AMAX)
        {
            for (int j = i + 1; j < n; ++j)
                t[j] += dt;
            sx = spline(t, x);
            sy = spline(t, y);
            aix = std::max(std::abs(sx.deriv(2,t[i]+0.01)),std::abs(sx.deriv(2,t[i+1]-0.01)));
            aiy = std::max(std::abs(sy.deriv(2,t[i]+0.01)),std::abs(sy.deriv(2,t[i+1]-0.01)));
            vix = set_v(sx, t[i], t[i+1], i);
            viy = set_v(sy, t[i], t[i+1], i);
        }


        std::cout<<"i = "<< i<<std::endl;
        std::cout<<"t = ";
        for (double i : t)
            std::cout << i <<" ";
        std::cout<<std::endl;
    }

    return t;
}


    double set_v(tk::spline s, double t1, double t2, int i)
    {
        double tmax = -(2 * s.m_c[i] - 6 * s.m_d[i] * s.m_x[i]) / (6 * s.m_d[i]);
        double ext = std::abs(s.deriv(1, tmax));
        double x1 = std::abs(s.deriv(1, s.m_x[i]));
        double x2 = std::abs(s.deriv(1, s.m_x[i+1]));

        if (tmax > t1 && tmax < t2)
            return ext;
        return std::max(x1, x2);
    }






    std::vector<double> Iset_time(std::vector<double> x,
                                 std::vector<double> y,
                                 double vmax, double amax)
    {
           int nx = size(x);
           int ny = size(y);
           assert(nx == ny);
           assert(nx > 2);
           int n = nx;
           std::vector<double> results = {0};
           double ti = 0;

           for (int i = 1; i<n; i++)
           {
               double tx = std::abs(x[i]-x[i-1])/(vmax);
               double ty = std::abs(y[i]-y[i-1])/(vmax);
               double tv = std::max(tx,ty);
               double tdx = sqrt(2*std::abs(x[i]-x[i-1])/amax);
               double tdy = sqrt(2*std::abs(y[i]-y[i-1])/amax);
               double ta = std::max(tdx,tdy);
               ti = ti + std::max(ta,tv);
               results.push_back(ti);
           }
           for (int j =1; j<n; j++)
           {
               assert(results[j]>results[j-1]);
           }
           assert(n == results.size());
           return results;
    }



};

// 3D spline interpolation
class Fspline
{
public:
    std::vector<double> x, y, z;        // x,y,z coordinates of points and date
    double amax = 0, vmax = 0;
    std::vector<double> t;
    tk::spline sx = spline(t,x);
    tk::spline sy = spline(t,y);
    tk::spline sz = spline(t,z);

    Fspline():
        x(),y(),z()
        {}

    Fspline(const std::vector<double>& X, const std::vector<double>& Y,
            const std::vector<double>& Z, double VMAX, double AMAX):
        x(X), y(Y), z(Z),t(temps(X, Y, Z, VMAX, AMAX))
        {
            sx = spline(t, X);
            sy = spline(t, Y);
            sz = spline(t, Z);
            amax = AMAX;
            vmax = VMAX;
            x = X;
            y = Y;
            z = Z;
            this->F_set_points();
        }

    void F_set_coeffs_from_b()    // calculate c_i, d_i from b_i
    {
        sx.set_coeffs_from_b();
        sy.set_coeffs_from_b();
        sz.set_coeffs_from_b();
    }

    const std::vector<size_t> F_find_closest(const std::vector<double> pos)
    {
        const std::vector<size_t> result =
        {
        sx.find_closest(pos[0]),
        sy.find_closest(pos[1]),
        sz.find_closest(pos[2])
        };
        return result;
    }

    // modify boundary conditions: if called it must be before F_set_points()
    void F_set_boundary()
    {
        sx.set_boundary(sx.m_left, sx.m_left_value, sx.m_right, sx.m_right_value);
        sy.set_boundary(sy.m_left, sy.m_left_value, sy.m_right, sy.m_right_value);
        sz.set_boundary(sz.m_left, sz.m_left_value, sz.m_right, sz.m_right_value);
    }

    // set all data points
    void F_set_points()
    {
        sx.set_points(t, x);
        sy.set_points(t, y);
        sz.set_points(t, z);
    }

    // evaluates the spline at a vector pos
    const std::vector<double> Foperator(spline sx, spline sy, spline sz, std::vector<double> pos)
    {
        std::vector<double> result =
        {
            sx.operator()(pos[0]),
            sy.operator()(pos[1]),
            sz.operator()(pos[2])
        };
        return result;
    }

    const std::vector<double> Fderiv(int order, double t) const
    {
        double dx = sx.deriv(order, t);
        double dy = sy.deriv(order, t);
        double dz = sz.deriv(order, t);
        return std::vector<double> {dx, dy, dz};
    }


std::vector<double> temps(std::vector<double> x,
                          std::vector<double> y,
                          std::vector<double> z,
                          double VMAX = 0, double AMAX = 0)
{
    std::vector<double> t = set_time(x, y, z, VMAX, AMAX);
    tk::spline sx, sy, sz;
    sx = spline(t, x);
    sy = spline(t, y);
    sz = spline(t, z);
    int n = t.size();
    double dt = 1/(2*VMAX);

    double aix = std::abs(sx.deriv(2,t[0]+0.01));
    double aiy = std::abs(sy.deriv(2,t[0]+0.01));
    double aiz = std::abs(sz.deriv(2,t[0]+0.01));

    while (aix > AMAX || aiy > AMAX || aiz > AMAX)
    {
        for (int j = 1; j < n; ++j)
            t[j] += dt;
        sx = spline(t, x);
        sy = spline(t, y);
        sz = spline(t, z);
        aix = std::abs(sx.deriv(2,t[0]+0.01));
        aiy = std::abs(sy.deriv(2,t[0]+0.01));
        aiz = std::abs(sz.deriv(2,t[0]+0.01));
    }


    for (int i = 0; i < n; i++)
    {
        double vix = set_v(sx, t[i], t[i+1], i);
        double viy = set_v(sy, t[i], t[i+1], i);
        double viz = set_v(sz, t[i], t[i+1], i);
        double aix = std::max(std::abs(sx.deriv(2,t[i]+0.01)),std::abs(sx.deriv(2,t[i+1]-0.01)));
        double aiy = std::max(std::abs(sy.deriv(2,t[i]+0.01)),std::abs(sy.deriv(2,t[i+1]-0.01)));
        double aiz = std::max(std::abs(sz.deriv(2,t[i]+0.01)),std::abs(sz.deriv(2,t[i+1]-0.01)));



        while (vix > VMAX || viy > VMAX || viz > VMAX || aix > AMAX || aiy > AMAX || aiz > AMAX)
        {
            for (int j = i + 1; j < n; ++j)
                t[j] += dt;
            sx = spline(t, x);
            sy = spline(t, y);
            sz = spline(t, z);
            aix = std::max(std::abs(sx.deriv(2,t[i]+0.01)),std::abs(sx.deriv(2,t[i+1]-0.01)));
            aiy = std::max(std::abs(sy.deriv(2,t[i]+0.01)),std::abs(sy.deriv(2,t[i+1]-0.01)));
            aiz = std::max(std::abs(sz.deriv(2,t[i]+0.01)),std::abs(sz.deriv(2,t[i+1]-0.01)));
            vix = set_v(sx, t[i], t[i+1], i);
            viy = set_v(sy, t[i], t[i+1], i);
            viz = set_v(sz, t[i], t[i+1], i);
            std::cout<<"di = ";
            for(int i = 0; i< sx.m_d.size(); i++)
                std::cout<<sx.m_d[i]<<" ";
            std::cout<<std::endl;
        }


        std::cout<<"i = "<< i<<std::endl;
        std::cout<<"t = ";
        for (double i : t)
            std::cout << i <<" ";
        std::cout<<std::endl;
    }

    return t;
}

    double set_v(tk::spline s, double t1, double t2, int i)
    {
        double tmax = t1 - s.m_c[i] / (3 * s.m_d[i]);
        double ext = std::abs(s.deriv(1, tmax));
        double x1 = std::abs(s.deriv(1, s.m_x[i]));
        double x2 = std::abs(s.deriv(1, s.m_x[i+1]));

        if (tmax > t1 && tmax < t2)
            return ext;
        return std::max(x1, x2);
    }


    std::vector<double> set_time(std::vector<double> x,
                                 std::vector<double> y,
                                 std::vector<double> z,
                                 double vmax, double amax)
    {
           int nx = size(x);
           int ny = size(y);
           int nz = size(z);
           assert(nx == ny);
           assert(nx == nz);
           assert(nx > 2);
           int n = nx;
           std::vector<double> results = {0};
           double ti = 0;

           for (int i = 1; i<n; i++)
           {
               double tx = std::abs(x[i]-x[i-1])/(vmax);
               double ty = std::abs(y[i]-y[i-1])/(vmax);
               double tz = std::abs(z[i]-z[i-1])/(vmax);
               double tv = std::max(tz, std::max(tx,ty));
               double tdx = sqrt(2*std::abs(x[i]-x[i-1])/amax);
               double tdy = sqrt(2*std::abs(y[i]-y[i-1])/amax);
               double tdz = sqrt(2*std::abs(z[i]-z[i-1])/amax);
               double ta = std::max(std::min(tdx,tdy),tdz);
               ti = ti + std::max(ta,tv);
               results.push_back(ti);
           }
           for (int j =1; j<n; j++)
           {
               assert(results[j]>results[j-1]);
           }
           assert(n == results.size());
           return results;
    }



};


namespace internal
{
// band matrix solver
class band_matrix
{
private:
    std::vector< std::vector<double> > c1;  // upper band
    std::vector< std::vector<double> > c2;  // lower band
public:
    band_matrix() {};                             // constructor
    band_matrix(int dim, int n_u, int n_l);       // constructor
    ~band_matrix() {};                            // destructor
    void resize(int dim, int n_u, int n_l);      // init with dim,n_u,n_l
    int dim() const;                             // matrix dimension
    int num_upper() const
    {
        return (int)c1.size()-1;
    }
    int num_lower() const
    {
        return (int)c2.size()-1;
    }
    // access operator
    double & operator () (int i, int j);            // write
    double   operator () (int i, int j) const;      // read
    // we can store an additional diagonal (in c2)
    double& saved_diag(int i);
    double  saved_diag(int i) const;
    void lu_decompose();
    std::vector<double> r_solve(const std::vector<double>& b) const;
    std::vector<double> l_solve(const std::vector<double>& b) const;
    std::vector<double> lu_solve(const std::vector<double>& b,
                                 bool is_lu_decomposed=false);

};
} // namespace internal





// spline implementation
// -----------------------

void spline::set_boundary(spline::bd_type left, double left_value,
                          spline::bd_type right, double right_value)
{
    assert(m_x.size()==0);          // set_points() must not have happened yet
    m_left=left;
    m_right=right;
    m_left_value=left_value;
    m_right_value=right_value;
}


void spline::set_coeffs_from_b()
{
    assert(m_x.size()==m_y.size());
    assert(m_x.size()==m_b.size());
    assert(m_x.size()>2);
    size_t n=m_b.size();
    if(m_c.size()!=n)
        m_c.resize(n);
    if(m_d.size()!=n)
        m_d.resize(n);

    for(size_t i=0; i<n-1; i++) {
        const double h  = m_x[i+1]-m_x[i];
        // from continuity and differentiability condition
        m_c[i] = ( 3.0*(m_y[i+1]-m_y[i])/h - (2.0*m_b[i]+m_b[i+1]) ) / h;
        // from differentiability condition
        m_d[i] = ( (m_b[i+1]-m_b[i])/(3.0*h) - 2.0/3.0*m_c[i] ) / h;
    }

    // for left extrapolation coefficients
    m_c0 = (m_left==first_deriv) ? 0.0 : m_c[0];
}

void spline::set_points(const std::vector<double>& x,
                        const std::vector<double>& y)
{
    assert(size(x)==size(y));
    assert(x.size()>2);
    m_x=x;
    m_y=y;
    int n = (int) x.size();
    for(int i=0; i<n-1; i++) {
        assert(m_x[i]<m_x[i+1]);
    }
        internal::band_matrix A(n,1,1);
        std::vector<double>  rhs(n);
        for(int i=1; i<n-1; i++) {
            A(i,i-1)=1.0/3.0*(x[i]-x[i-1]);
            A(i,i)=2.0/3.0*(x[i+1]-x[i-1]);
            A(i,i+1)=1.0/3.0*(x[i+1]-x[i]);
            rhs[i]=(y[i+1]-y[i])/(x[i+1]-x[i]) - (y[i]-y[i-1])/(x[i]-x[i-1]);
        }

    // boundary conditions
    if(m_left == spline::second_deriv) {
        // 2*c[0] = f''
        A(0,0)=2.0;
        A(0,1)=0.0;
        rhs[0]=m_left_value;
    }

    else if(m_left == spline::first_deriv) {
       // b[0] = f', needs to be re-expressed in terms of c:
       // (2c[0]+c[1])(x[1]-x[0]) = 3 ((y[1]-y[0])/(x[1]-x[0]) - f')
       A(0,0)=2.0*(x[1]-x[0]);
       A(0,1)=1.0*(x[1]-x[0]);
       rhs[0]=3.0*((y[1]-y[0])/(x[1]-x[0])-m_left_value);
    }

    else {
        assert(false);
    }

    if(m_right == spline::second_deriv) {
         // 2*c[n-1] = f''
         A(n-1,n-1)=2.0;
         A(n-1,n-2)=0.0;
         rhs[n-1]=m_right_value;
    }

    else if(m_right == spline::first_deriv) {
       // b[n-1] = f', needs to be re-expressed in terms of c:
       // (c[n-2]+2c[n-1])(x[n-1]-x[n-2])
       // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
       A(n-1,n-1)=2.0*(x[n-1]-x[n-2]);
       A(n-1,n-2)=1.0*(x[n-1]-x[n-2]);
       rhs[n-1]=3.0*(m_right_value-(y[n-1]-y[n-2])/(x[n-1]-x[n-2]));
    }

    // solve the equation system to obtain the parameters c[]
    m_c=A.lu_solve(rhs);

    // calculate parameters b[] and d[] based on c[]
    m_d.resize(n);
    m_b.resize(n);
    for(int i=0; i<n-1; i++) {
        m_d[i]=1.0/3.0*(m_c[i+1]-m_c[i])/(x[i+1]-x[i]);
        m_b[i]=(y[i+1]-y[i])/(x[i+1]-x[i])- 1.0/3.0*(2.0*m_c[i]+m_c[i+1])*(x[i+1]-x[i]);
    }
    // for the right extrapolation coefficients (zero cubic term)
    // f_{n-1}(x) = y_{n-1} + b*(x-x_{n-1}) + c*(x-x_{n-1})^2
    double h=x[n-1]-x[n-2];
    // m_c[n-1] is determined by the boundary condition
    m_d[n-1]=0.0;
    m_b[n-1]=3.0*m_d[n-2]*h*h+2.0*m_c[n-2]*h+m_b[n-2];   // = f'_{n-2}(x_{n-1})
    if(m_right==first_deriv)
        m_c[n-1]=0.0;   // force linear extrapolation


    // for left extrapolation coefficients
    m_c0 = (m_left==first_deriv) ? 0.0 : m_c[0];
}


// return the closest idx so that m_x[idx] <= x (return 0 if x<m_x[0])
size_t spline::find_closest(double x) const
{
    std::vector<double>::const_iterator it;
    it=std::upper_bound(m_x.begin(),m_x.end(),x);       // *it > x
    size_t idx = std::max( int(it-m_x.begin())-1, 0);   // m_x[idx] <= x
    return idx;
}

double spline::operator() (double x) const
{
    // polynomial evaluation using Horner's scheme
    // TODO: consider more numerically accurate algorithms, e.g.:
    //   - Clenshaw
    //   - Even-Odd method by A.C.R. Newbery
    //   - Compensated Horner Scheme
    size_t n=m_x.size();
    size_t idx=find_closest(x);

    double h=x-m_x[idx];
    double interpol;
    if(x<m_x[0]) {
        // extrapolation to the left
        interpol=(m_c0*h + m_b[0])*h + m_y[0];
    } else if(x>m_x[n-1]) {
        // extrapolation to the right
        interpol=(m_c[n-1]*h + m_b[n-1])*h + m_y[n-1];
    } else {
        // interpolation
        interpol=((m_d[idx]*h + m_c[idx])*h + m_b[idx])*h + m_y[idx];
    }
    return interpol;
}

double spline::deriv(int order, double x) const
{
    assert(order>0);
    size_t n=m_x.size();
    size_t idx = find_closest(x);

    double h=x-m_x[idx];
    double interpol;
    if(x<m_x[0]) {
        // extrapolation to the left
        switch(order) {
        case 1:
            interpol=2.0*m_c0*h + m_b[0];
            break;
        case 2:
            interpol=2.0*m_c0;
            break;
        default:
            interpol=0.0;
            break;
        }
    } else if(x>m_x[n-1]) {
        // extrapolation to the right
        switch(order) {
        case 1:
            interpol=2.0*m_c[n-1]*h + m_b[n-1];
            break;
        case 2:
            interpol=2.0*m_c[n-1];
            break;
        default:
            interpol=0.0;
            break;
        }
    } else {
        // interpolation
        switch(order) {
        case 1:
            interpol=(3.0*m_d[idx]*h + 2.0*m_c[idx])*h + m_b[idx];
            break;
        case 2:
            interpol=6.0*m_d[idx]*h + 2.0*m_c[idx];
            break;
        case 3:
            interpol=6.0*m_d[idx];
            break;
        default:
            interpol=0.0;
            break;
        }
    }
    return interpol;
}



namespace internal
{

// band_matrix implementation
// -------------------------

band_matrix::band_matrix(int dim, int n_u, int n_l)
{
    resize(dim, n_u, n_l);
}
void band_matrix::resize(int dim, int n_u, int n_l)
{
    assert(dim>0);
    assert(n_u>=0);
    assert(n_l>=0);
    c1.resize(n_u+1);
    c2.resize(n_l+1);
    for(size_t i=0; i<c1.size(); i++) {
        c1[i].resize(dim);
    }
    for(size_t i=0; i<c2.size(); i++) {
        c2[i].resize(dim);
    }
}
int band_matrix::dim() const
{
    if(c1.size()>0) {
        return c1[0].size();
    } else {
        return 0;
    }
}


// defines the new operator (), so that we can access the elements
// by A(i,j), index going from i=0,...,dim()-1
double & band_matrix::operator () (int i, int j)
{
    int k=j-i;       // what band is the entry
    assert( (i>=0) && (i<dim()) && (j>=0) && (j<dim()) );
    assert( (-num_lower()<=k) && (k<=num_upper()) );
    // k=0 -> diagonal, k<0 lower left part, k>0 upper right part
    if(k>=0)    return c1[k][i];
    else        return c2[-k][i];
}

double band_matrix::operator () (int i, int j) const
{
    int k=j-i;       // what band is the entry
    assert( (i>=0) && (i<dim()) && (j>=0) && (j<dim()) );
    assert( (-num_lower()<=k) && (k<=num_upper()) );
    // k=0 -> diagonal, k<0 lower left part, k>0 upper right part
    if(k>=0)    return c1[k][i];
    else        return c2[-k][i];
}

// second diag (used in LU decomposition), saved in c2
double band_matrix::saved_diag(int i) const
{
    assert( (i>=0) && (i<dim()) );
    return c2[0][i];
}

double & band_matrix::saved_diag(int i)
{
    assert( (i>=0) && (i<dim()) );
    return c2[0][i];
}

// LR-Decomposition of a band matrix
void band_matrix::lu_decompose()
{
    int  i_max,j_max;
    int  j_min;
    double x;

    // preconditioning
    // normalize column i so that a_ii=1
    for(int i=0; i<this->dim(); i++) {
        assert(this->operator()(i,i)!=0.0);
        this->saved_diag(i)=1.0/this->operator()(i,i);
        j_min=std::max(0,i-this->num_lower());
        j_max=std::min(this->dim()-1,i+this->num_upper());
        for(int j=j_min; j<=j_max; j++) {
            this->operator()(i,j) *= this->saved_diag(i);
        }
        this->operator()(i,i)=1.0;          // prevents rounding errors
    }

    // Gauss LR-Decomposition
    for(int k=0; k<this->dim(); k++) {
        i_max=std::min(this->dim()-1,k+this->num_lower());  // num_lower not a mistake!
        for(int i=k+1; i<=i_max; i++) {
            assert(this->operator()(k,k)!=0.0);
            x=-this->operator()(i,k)/this->operator()(k,k);
            this->operator()(i,k)=-x;                         // assembly part of L
            j_max=std::min(this->dim()-1,k+this->num_upper());
            for(int j=k+1; j<=j_max; j++) {
                // assembly part of R
                this->operator()(i,j)=this->operator()(i,j)+x*this->operator()(k,j);
            }
        }
    }
}

// solves Ly=b
std::vector<double> band_matrix::l_solve(const std::vector<double>& b) const
{
    assert( this->dim()==(int)b.size() );
    std::vector<double> x(this->dim());
    int j_start;
    double sum;
    for(int i=0; i<this->dim(); i++) {
        sum=0;
        j_start=std::max(0,i-this->num_lower());
        for(int j=j_start; j<i; j++) sum += this->operator()(i,j)*x[j];
        x[i]=(b[i]*this->saved_diag(i)) - sum;
    }
    return x;
}

// solves Rx=y
std::vector<double> band_matrix::r_solve(const std::vector<double>& b) const
{
    assert( this->dim()==(int)b.size() );
    std::vector<double> x(this->dim());
    int j_stop;
    double sum;
    for(int i=this->dim()-1; i>=0; i--) {
        sum=0;
        j_stop=std::min(this->dim()-1,i+this->num_upper());
        for(int j=i+1; j<=j_stop; j++) sum += this->operator()(i,j)*x[j];
        x[i]=( b[i] - sum ) / this->operator()(i,i);
    }
    return x;
}

std::vector<double> band_matrix::lu_solve(const std::vector<double>& b, bool is_lu_decomposed)
{
    assert( this->dim()==(int)b.size() );
    std::vector<double>  x,y;
    if(is_lu_decomposed==false) {
        this->lu_decompose();
    }
    y=this->l_solve(b);
    x=this->r_solve(y);
    return x;
}

} // namespace internal


} // namespace tk



#endif


/*
This library provides three cpp classes named respectively spline, Ispline and Fspline.

--------------------------------
First : the "spline" class
--------------------------------

It takes in argument :
- two vectors named m_x and m_y which are the coordinates of waypoints.
- two boarders conditions m_left_value and m_right_value that you can set (if not their value is set on 0)
- two arguments to know if you want to set the boundary limits two the first or the second derivative m_left or m_right.
(their arguments are first_deriv and second_deriv)

==> Thus, the constructor is call like so:
        S = spline(m_x, m_y, m_left, m_left_value, m_right, m_right_value)

It returns :
- m_b, m_c and m_d three vectors of polynomial coeficients that interpolate the trajectory between m_x and m_y using splines.

The function provided by the class:

- set_boundary(spline::bd_type left, double left_value,
               spline::bd_type right, double right_value)
===> set the boundary limits of the spline

- set_points(const std::vector<double>& x,
             const std::vector<double>& y)
===>calculate the spline for two vectors

- find_closest(double x)
===> give the closest index to x in order that m_x[i] <= x

- deriv(int order, double x)
===> derivate the spline at the order that you want at the x that you want

-----------------------------------
second class : Ispline
-----------------------------------

It takes in argument :
- two vectors named x and y which are the coordinates of waypoints.
- two maximal value for the first and second derivative vmax and amax.

==> Thus, the constructor is call like so:
        IS = Ispline(m_x, m_y, vmax, amax)

It returns :
- a vector t, the smallest time in which the interpolation is able to be smaller than vmax and amax in module.
- two splines that are built on t : sx and sy

The function provided by the class:

- I_set_boundary(spline::bd_type left, double left_value,
               spline::bd_type right, double right_value)
===> set the boundary limits of the spline.

- I_set_points(const std::vector<double>& x,
             const std::vector<double>& y)
===> calculate the spline for two vectors.

- Itemps(std::vector<double> x, std::vector<double> y, double VMAX = 0, double AMAX = 0)
===> set a vector t of dates that makes possible the interpolation by respecting amax and vmax.


------------------------------------
third class: Fspline
------------------------------------

it does almost the same thing that the previous class but it have a third z vector of positions/
*/
