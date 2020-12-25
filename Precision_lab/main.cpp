#include <iostream>
#include <iomanip> // Только для вывода числа
#include <cmath> // Working with double only
#include <limits>

class DoubleDouble;

int PRECISION = 15;
int SPLITCONSTANT = 27;

using namespace std;

DoubleDouble Fast2Sum(double x, double y);
DoubleDouble Mult(double x, double y);
DoubleDouble factInv(unsigned int n);
DoubleDouble Split(double x, int s);

class DoubleDouble
{
public:
  double h, l;


    DoubleDouble(double h_inp = 0, double l_inp = 0)
    {
        h = h_inp;
        l = l_inp;
    }

    DoubleDouble(const DoubleDouble & dd)
    {
        l = dd.l;
        h = dd.h;
    }

    DoubleDouble & operator=(const DoubleDouble & dd)
    {
        if (this != &dd)
        {
            l = dd.l;
            h = dd.h;
        }

        return *this;
    }

    bool isEqual(DoubleDouble y)
    {
        return ((l == y.l) && (h == y.h));
    }

    bool isFinite()
    {
        return (isfinite(l) && isfinite(h));
    }

    bool isSmaller(DoubleDouble y)
    {
        if (h > y.h)
            return false;
        if (l > y.l)
            return false;
        return true;
    }

    DoubleDouble minus()
    {
        return DoubleDouble(-h, -l);
    }

    DoubleDouble abs()
    {
        DoubleDouble res(h, l);
        if (h < 0)
            res.h = -h;
        if (l < 0)
            res.l = -l;
        return res;
    }

    void view()
    {
        cout << setprecision(PRECISION) << "h = " << h << '\n';
        cout << setprecision(PRECISION) << "l = " << l << '\n';
    }

    DoubleDouble add(DoubleDouble y)
    {
        DoubleDouble r;
        double s;
        if (h >= y.h) {
            r = Fast2Sum(h, y.h);
            s = (r.l + y.l) + l;
        } else {
            r = Fast2Sum(y.h, h);
            s = (r.l + l) + y.l;
        }
    
        DoubleDouble res;
        res = Fast2Sum(r.h, s);
        return res;
    }

    DoubleDouble mult(DoubleDouble y)
    {
        DoubleDouble c = Mult(h, y.h);
        double p1 = h * y.l;
        double p2 = l * y.h;
        c.l = c.l + (p1 + p2);
        DoubleDouble res = Fast2Sum(c.h, c.l);
        return res;
    }

    DoubleDouble pow(unsigned int n)
    {
        if (n == 0)
            return DoubleDouble(1, 0);

        DoubleDouble copy(h, l);

        if (n == 1)
            return copy;

        return copy.mult(copy.pow(n-1));
    }

    DoubleDouble sinTaylorPartNumber(unsigned int n)
    {
        if (n == 0)
            return DoubleDouble(0, 0);

        DoubleDouble copy = *this;

        DoubleDouble fInv = factInv(2*n -1);

        copy = copy.pow(2*n -1);
        copy = copy.mult(fInv);

        if (n%2 == 0)
            copy = copy.minus();
        
        return copy;
    }

    DoubleDouble sin()
    {
        DoubleDouble PI = Split(M_PI, SPLITCONSTANT);
        DoubleDouble copy(h, l); 
        // h > PI
        if (PI.isSmaller(*this))
            return copy.add(PI.minus()).sin().minus();

        // -h > PI
        if (isSmaller(PI.minus()))
            return copy.add(PI).sin().minus();

        DoubleDouble oldRes(std::numeric_limits<double>::max()), newRes;
        DoubleDouble oldDif, newDif;

        bool converged, legalValue, contToConverge;

        unsigned int pre = 0;

        do
        {
            copy = *this;

            oldDif = newDif;
            oldRes = newRes;
            pre++;

            copy = copy.sinTaylorPartNumber(pre);

            newRes = copy.add(oldRes);

            newDif = copy.abs();
            
            converged = newRes.isEqual(oldRes);
            legalValue = newRes.isFinite();
            contToConverge = newDif.isSmaller(oldDif) || pre < 10;



            cout << "Current sin value\n";
            newRes.view();
            cout << '\n';
            

        } while (!converged && legalValue && contToConverge);

        if (converged)
            cout << "Converged\n";
        if (legalValue)
            cout << "next is illegal value\n";
        if (!contToConverge)
            cout << "Stopped conversion\n";

        return oldRes;
    }
};

DoubleDouble Fast2Sum(double x, double y)
{
  double s = x + y;
  double z = s - x;
  double t = y - z;
  return DoubleDouble(s, t);
}

DoubleDouble Split(double x, int s)
{
    long int C = (1 << s) + 1;
    double t1 = C*x;
    double t2 = x - t1;
    double r1 = t1 + t2;
    double r2 = x - r1;
    return DoubleDouble(r1, r2);
}

DoubleDouble Mult(double x, double y)
{
    int s = 27;
    DoubleDouble dx = Split(x, s);
    DoubleDouble dy = Split(y, s);

    double r1 = x*y;
    double t1 = -r1 + (dx.h * dy.h);
    double t2 = t1 + (dx.h * dy.l);
    double t3 = t2 + (dx.l * dy.h);
    double r2 = t3 + (dx.l * dy.l);
        
    return DoubleDouble(r1, r2);
}

DoubleDouble factInv(unsigned int n)
{
    if (n == 1)
        return DoubleDouble(1, 0);
    DoubleDouble dnInv = Split(1.0/n, SPLITCONSTANT);
    return dnInv.mult(factInv(n-1));
}

int main()
{
    // DoubleDouble PI = Split(M_PI, SPLITCONSTANT);
    // cout << "M_PI = " << setprecision(PRECISION*5) << M_PI << '\n';
    // cout << "Programm PI\n";
    // PI.view();
    DoubleDouble d = Split(2, SPLITCONSTANT);
    cout << "d\n";
    d.view();

    // for (int i=1; i<20; i++)
    // {
    //     cout << "Part " << i << '\n';
    //     d.sinTaylorPartNumber(i).view();
    //     cout << '\n';
    // }

    DoubleDouble res = d.sin();

    cout << "RESULT\n";
    res.view();
    cout << "h + l == " << setprecision(PRECISION) << res.h + res.l << '\n';
}

// sin(100) == -0.506365641109758793656557610459785432065032721290657323443392473...
// sin(2) == 0.9092974268256816953960198659117448427022549714478902683789...


// sin(100) by the programm ==
// h = -0.506460370711553919953473723581
// l = -9.82362775736079696677550152375e-18