#include "AMPCore.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"

class myWaveFront : public amp::PointWaveFrontAlgorithm{
    public:

    myWaveFront(){};
    virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;
};