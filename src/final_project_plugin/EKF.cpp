#include <Eigen/Dense>

class EKF
{
public:
    EKF(double x, double y, double vx, double vy, double dt)
    {
        xk(0) = x;
        xk(1) = y;
        xk(2) = vx;
        xk(3) = vy;

        initA(dt);
        initB(dt);
        initH(dt);
        initQ();
        initR();
        initI();
    }

    void Update(double x, double y, double ax, double ay)
    {
        Eigen::Vector2d u;
        u << ax, ay;

        Eigen::Vector2d z;
        z << x, y;

        Eigen::Vector4d x_minus = A * xk + B * u;
        Eigen::Matrix4d P_minus = A * Pk * A.transpose() + Q;
        Eigen::Matrix<double, 4, 2> K = P_minus * H.transpose() * (H * P_minus * H.transpose() + R).inverse();
        xk = K * (z - H * x_minus);
        Pk = (I - K * H) * P_minus;
    }

    std::tuple<double, double, double, double> GetState()
    {
        return std::make_tuple(xk(0), xk(1), xk(2), xk(3));
    }

private:
    void initA(double dt)
    {
        A << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
    }

    void initB(double dt)
    {
        B << (dt * dt) / 2, 0,
            0, (dt * dt) / 2,
            dt / 2, 0,
            0, dt / 2;
    }

    void initH(double dt)
    {
        H.Identity();
    }

    void initQ()
    {
        Q << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
    }

    void initR()
    {
        R << 0, 0,
            0, 0;
    }

    void initI()
    {
        I.Identity();
    }

    Eigen::Matrix4d A;
    Eigen::Matrix<double, 4, 2> B;
    Eigen::Matrix<double, 2, 4> H;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix4d I;
    Eigen::Vector4d xk;
    Eigen::Matrix4d Pk;
};