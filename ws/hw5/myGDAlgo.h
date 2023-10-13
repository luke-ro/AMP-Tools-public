#include "AMPCore.h"
#include "hw/HW5.h"
#include "Helpers.h"

class myGDAlgo : public amp::GDAlgorithm{
    public:

    myGDAlgo(int sx0 = 100, int sx1 = 100, double dstar_goal=5.0, double zeta=1.0, double Qstar=0.5, double eta=1.0);

    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    private:
    int _sx0;
    int _sx1;
    double _dstar_goal;
    double _zeta;
    double _Qstar;
    double _eta;
    // _grad
};