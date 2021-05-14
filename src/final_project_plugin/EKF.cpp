#include <Eigen/Dense>
#include <gazebo/gazebo.hh>

struct EKFState
{
    double x;
    double y;
    double vx;
    double vy;
};

namespace gazebo
{
    class EKF
    {
    public:
        EKF(double x, double y, double vx, double vy, bool withCamera)
        {
            xk(0) = x;
            xk(1) = y;
            xk(2) = vx;
            xk(3) = vy;

            initQ();
            initI();
            initP();
            initR(withCamera);

            this->withCamera = withCamera;
        }

        void Update(double x, double y, double ax, double ay, double dt)
        {
            assert(!this->withCamera);

            initA(dt);
            initB(dt);
            initH(dt, false);

            Eigen::Vector2d u;
            u << ax, ay;

            Eigen::Vector2d z;
            z << x, y;

            Eigen::Vector4d x_minus = A * xk + B * u;
            Eigen::Matrix4d P_minus = A * Pk * A.transpose() + Q;
            Eigen::Matrix<double, 4, 2> K = P_minus * H.transpose() * (H * P_minus * H.transpose() + R).inverse();
            xk = x_minus + K * (z - H * x_minus);
            Pk = (I - K * H) * P_minus;
        }

        void UpdateWithCamera(double x, double y, double ax, double ay, double dt, double cameraX, double cameraY)
        {
            assert(this->withCamera);

            initA(dt);
            initB(dt);
            initH(dt, true);

            Eigen::Vector2d u;
            u << ax, ay;

            Eigen::Vector4d z;
            z << x, y, cameraX, cameraY;

            Eigen::Vector4d x_minus = A * xk + B * u;
            Eigen::Matrix4d P_minus = A * Pk * A.transpose() + Q;
            Eigen::Matrix<double, 4, 4> K = P_minus * HwithCamera.transpose() * (HwithCamera * P_minus * HwithCamera.transpose() + RwithCamera).inverse();
            xk = x_minus + K * (z - HwithCamera * x_minus);
            Pk = (I - K * HwithCamera) * P_minus;
        }

        EKFState GetState()
        {
            EKFState state;
            state.x = xk(0);
            state.y = xk(1);
            state.vx = xk(2);
            state.vy = xk(3);

            return state;
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
                dt, 0,
                0, dt;
        }

        void initH(double dt, bool withCamera)
        {
            if (withCamera)
            {
                HwithCamera << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    1, 0, 0, 0,
                    0, 1, 0, 0;
            }
            else
            {
                H << 1, 0, 0, 0,
                    0, 1, 0, 0;
            }
        }

        void initQ()
        {
            Q << 0.00000025, 0, 0, 0,
                0, 0.00000025, 0, 0,
                0, 0, 0.00000025, 0,
                0, 0, 0, 0.00000025;
        }

        void initR(bool withCamera)
        {
            if (withCamera)
            {
                RwithCamera << 0.005625, 0, 0, 0,
                    0, 0.005625, 0, 0,
                    0, 0, 0.1, 0,
                    0, 0, 0, 0.1;
            }
            else
            {
                R << 0.005625, 0,
                    0, 0.005625;
            }
        }

        void initI()
        {
            I << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        }

        void initP()
        {
            Pk.Identity();
        }

        Eigen::Matrix4d A;
        Eigen::Matrix<double, 4, 2> B;
        Eigen::Matrix<double, 2, 4> H;
        Eigen::Matrix<double, 4, 4> HwithCamera;
        Eigen::Matrix4d Q;
        Eigen::Matrix2d R;
        Eigen::Matrix4d RwithCamera;
        Eigen::Matrix4d I;
        Eigen::Vector4d xk;
        Eigen::Matrix4d Pk;

        bool withCamera;
    };
}