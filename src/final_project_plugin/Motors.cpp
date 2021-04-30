#include <gazebo/common/PID.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

namespace gazebo
{
    class Control
    {
    public:
        Control()
        {
            this->rotorVelocitySlowdownSim = Control::kDefaultRotorVelocitySlowdownSim;
            this->frequencyCutoff = Control::kDefaultFrequencyCutoff;
            this->samplingRate = Control::kDefaultSamplingRate;

            this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
        }

        void LoadSdf(physics::ModelPtr model, sdf::ElementPtr controlSdf)
        {
            multiplier = controlSdf->Get<double>("multiplier");
            offset = controlSdf->Get<double>("offset");
            rotorVelocitySlowdownSim = controlSdf->Get<double>("controlVelocitySlowdownSim");
            filter.Fc(frequencyCutoff, samplingRate);
            filter.Set(0.0);

            pid.SetPGain(controlSdf->Get("p_gain", pid.GetPGain()).first);
            pid.SetIGain(controlSdf->Get("i_gain", pid.GetIGain()).first);
            pid.SetDGain(controlSdf->Get("d_gain", pid.GetDGain()).first);
            pid.SetIMax(controlSdf->Get("i_max", pid.GetIMax()).first);
            pid.SetIMin(controlSdf->Get("i_min", pid.GetIMin()).first);
            pid.SetCmdMax(controlSdf->Get("cmd_max", pid.GetCmdMax()).first);
            pid.SetCmdMin(controlSdf->Get("cmd_min", pid.GetCmdMin()).first);
            pid.SetCmd(0.0);

            std::string jointName = controlSdf->Get<std::string>("jointName");
            joint = model->GetJoint(jointName);
        }

        void ApplyCommand(double command, double dt)
        {
            assert(-1.0 <= command && command <= 1.0);

            double velocityTarget = (multiplier * (offset + command)) / rotorVelocitySlowdownSim;
            double velocity = joint->GetVelocity(0);
            double error = velocity - velocityTarget;
            double force = pid.Update(error, dt);
            joint->SetForce(0, force);
        }

    private:
        double rotorVelocitySlowdownSim;
        double frequencyCutoff;
        double samplingRate;
        double multiplier;
        double offset;

        common::PID pid;
        ignition::math::OnePole<double> filter;
        physics::JointPtr joint;

        static double kDefaultRotorVelocitySlowdownSim;
        static double kDefaultFrequencyCutoff;
        static double kDefaultSamplingRate;
    };

    double Control::kDefaultRotorVelocitySlowdownSim = 10.0;
    double Control::kDefaultFrequencyCutoff = 5.0;
    double Control::kDefaultSamplingRate = 0.2;
}