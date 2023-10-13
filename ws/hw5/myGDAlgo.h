#include "AMPCore.h"
#include "hw/HW5.h"
#include "Helpers.h"

class myGDAlgo : public amp::GDAlgorithm{
    public:

    myGDAlgo(double epsilon=0.5, double dstar_goal=5.0, double zeta=1.0, double Qstar=0.5, double eta=1.0, double alpha=0.1, int sx0 = 100, int sx1 = 100);

    virtual amp::Path2D plan(const amp::Problem2D& problem) override;
    void fillGradient(const amp::Problem2D& problem, std::vector<std::vector<Eigen::Vector2d>> &grad_arr);
    Eigen::Vector2d calcGrad(const amp::Problem2D& problem, const Eigen::Vector2d& q);
    Eigen::Vector2d gradUatt(const amp::Problem2D& problem, Eigen::Vector2d q);
    Eigen::Vector2d gradUrep(const amp::Problem2D& problem, Eigen::Vector2d q);

    private:
    double _dstar_goal;
    double _zeta;
    double _Qstar;
    double _eta;
    double _epsilon;
    double _alpha;
    int _sx0;
    int _sx1;
    // _grad
};