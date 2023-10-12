#include "AMPCore.h"
#include "hw/HW5.h"

class myGDAlgo : public amp::GDAlgorithm{
    public:

    myGDAlgo(int sx0 = 100, int sx1 = 100);

    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    private:
    int _sx0;
    int _sx1;
    // _grad
};