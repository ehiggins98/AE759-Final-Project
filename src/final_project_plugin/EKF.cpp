#include <Eigen/Dense>

class EKF
{
public:
    EKF(double x, double y, double vx, double vy)
    {
        state(0) = x;
        state(1) = y;
        state(2) = vx;
        state(3) = vy;
    }

    std::tuple<double, double, double, double> GetState()
    {
        return std::make_tuple(state(0), state(1), state(2), state(3));
    }

private:
    Eigen::Vector4d state;
};