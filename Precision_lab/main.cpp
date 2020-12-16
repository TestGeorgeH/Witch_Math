#include <iostream>
#include <iomanip> // Только для вывода числа
#include <cmath> // Working with double only
#include <limits>

int PRECISION = 30;
long double PI = 3.14159265358979323846264338327950288419716939937510L;

using namespace std;

class DoubleDouble;

DoubleDouble Fast2Sum(double x, double y);
DoubleDouble Fast2MultFMA(double x, double y);
int FACT(unsigned int n);

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
        DoubleDouble c = Fast2MultFMA(h, y.h);
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

    DoubleDouble sin()
    {
        DoubleDouble copy(h, l); 
        if (h > PI)
            return copy.add(-PI).mult(-1).sin();

        if (-h > PI)
            return copy.add(PI).mult(-1).sin();

        DoubleDouble oldRes(std::numeric_limits<double>::max()), newRes;
        DoubleDouble oldDif, newDif;

        bool converged, legalValue, contToConverge;

        unsigned int pre = 0;

        do
        {
            copy.h = h;
            copy.l = l;

            oldDif = newDif;
            oldRes = newRes;
            pre++;

            double fact = FACT(2*pre -1);
            double factInvd = 1/fact;
            DoubleDouble factInv(factInvd);

            copy = copy.pow(2*pre -1);

            copy = copy.mult(factInv);

            if (pre%2 == 0)
                copy = copy.mult(-1);

            newRes = copy.add(oldRes);

            newDif = copy.abs();

            converged = newRes.isEqual(oldRes);
            legalValue = newRes.isFinite();
            contToConverge = newDif.isSmaller(oldDif) || pre < 3;

        } while (!converged && legalValue && contToConverge);

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

DoubleDouble Fast2MultFMA(double x, double y)
{
  double r1 = x*y;
  double r2 = x*y - r1;
  return DoubleDouble(r1, r2);
}

int FACT(unsigned int n)
{
  if (n == 1)
    return 1;
  return n * FACT(n-1);
}


int main()
{
    DoubleDouble d(100);
    d.sin().view();
}