#include <iostream>
#include <iomanip> // Только для вывода числа
#include <cmath> // Working with double only
#include <limits>

int PRECISION = 15;
int SPLITCONSTANT = 27;

using namespace std;

class DoubleDouble
{
public:
  double h, l;


    DoubleDouble(double h_inp = 0, double l_inp = 0);
    DoubleDouble(const DoubleDouble & dd);
    DoubleDouble & operator=(const DoubleDouble & dd);

    bool isEqual(DoubleDouble y);

    bool isFinite();
    bool isSmaller(DoubleDouble y);

    DoubleDouble minus();

    DoubleDouble abs();

    void view();

    DoubleDouble add(DoubleDouble y);
    DoubleDouble mult(DoubleDouble y);

    DoubleDouble pow(unsigned int n);
    DoubleDouble sinTaylorPartNumber(unsigned int n);

    DoubleDouble sin();
};

DoubleDouble PI(3.141592653589793116e+00, 1.224646799147353207e-16);

DoubleDouble Fast2Sum(double x, double y);
DoubleDouble Split(double x, int s);
DoubleDouble Mult(double x, double y);
DoubleDouble factInv(unsigned int n);
