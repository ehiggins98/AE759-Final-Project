#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <unordered_map>
#include <random>

namespace gazebo
{
    class SimWirelessReceiver
    {
    public:
        SimWirelessReceiver(physics::ModelPtr model, std::unordered_map<std::string, physics::ModelPtr> transmitters)
        {
            this->model = model;
            this->transmitters = transmitters;
            this->generator = boost::shared_ptr<std::default_random_engine>(new std::default_random_engine());
            this->distribution = boost::shared_ptr<std::normal_distribution<double>>(new std::normal_distribution<double>(0, 0.075));
        }

        std::unordered_map<std::string, double> Sample()
        {
            std::unordered_map<std::string, double> result;

            ignition::math::v4::Pose3d receiverPose = model->WorldPose();
            for (std::unordered_map<std::string, physics::ModelPtr>::const_iterator itr = transmitters.cbegin(), end = transmitters.cend(); itr != end; itr++)
            {
                double trueDistance = receiverPose.CoordPositionSub(itr->second->WorldPose()).Length();
                double noise = (*distribution)(*generator);
                result.insert(std::pair<std::string, double>(itr->first, trueDistance + noise));
            }

            return result;
        }

    private:
        physics::ModelPtr model;
        std::unordered_map<std::string, physics::ModelPtr> transmitters;
        boost::shared_ptr<std::default_random_engine> generator;
        boost::shared_ptr<std::normal_distribution<double>> distribution;
    };
}