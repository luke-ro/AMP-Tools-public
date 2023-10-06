#include "AMPCore.h"
#include "CSpace2D.h"
#include "hw/HW4.h"

class MyGridCon : public amp::GridCSpace2DConstructor {
    public:
    
    virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;
};
