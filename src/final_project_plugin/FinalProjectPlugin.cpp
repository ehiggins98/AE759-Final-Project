#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/sensors.hh>

#include <Eigen/Dense>

#include "./Motors.cpp"
#include "./SimWirelessReceiver.cpp"

namespace gazebo
{
    class FinalProjectPlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&FinalProjectPlugin::OnUpdate, this));

            LoadSensors(_parent, _sdf);

            LoadControl(_parent, _sdf->GetElement("control"));
        }

        void LoadSensors(physics::ModelPtr model, sdf::ElementPtr _sdf)
        {
            std::string wirelessName = _sdf->Get("wirelessName", static_cast<std::string>("wireless_sensor")).first;
            std::string imuName = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;

            std::vector<std::string> wirelessScopedName = this->model->SensorScopedName(wirelessName);
            std::vector<std::string>
                imuScopedName = this->model->SensorScopedName(imuName);

            if (wirelessScopedName.size() > 1)
            {
                gzwarn << "Multiple names match " << wirelessName << ". Failing.\n";
            }
            else
            {
                sensors::WirelessReceiverPtr receiver = std::dynamic_pointer_cast<sensors::WirelessReceiver>(sensors::SensorManager::Instance()->GetSensor(wirelessScopedName[0]));
                std::unordered_map<std::string, physics::ModelPtr> transmitters = LoadTransmitters(model, _sdf);
                this->receiver = boost::shared_ptr<SimWirelessReceiver>(new SimWirelessReceiver(receiver, transmitters));
            }

            if (imuScopedName.size() > 1)
            {
                gzwarn << "Multiple names match " << imuName << ". Failing.\n";
            }
            else
            {
                this->imu = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor(imuScopedName[0]));
            }
        }

        std::unordered_map<std::string, physics::ModelPtr> LoadTransmitters(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            std::unordered_map<std::string, physics::ModelPtr> transmitters;

            sdf::ElementPtr transmittersSdf = sdf->GetElement("transmitter");
            while (transmittersSdf)
            {
                gzmsg << "hi"
                      << "\n";
                if (!transmittersSdf->HasAttribute("name"))
                {
                    break;
                }

                std::string sensorName = transmittersSdf->GetAttribute("name")->GetAsString();
                transmitters.insert(std::pair<std::string, physics::ModelPtr>(sensorName, model->NestedModel(sensorName)));
                transmittersSdf = transmittersSdf->GetNextElement();
            }

            return transmitters;
        }

        void LoadControl(physics::ModelPtr model, sdf::ElementPtr controlSdf)
        {
            while (controlSdf)
            {
                Control control;
                control.LoadSdf(model, controlSdf);
                controls.push_back(control);
                controlSdf = controlSdf->GetNextElement("control");
            }
        }

        void OnUpdate()
        {
            receiver->Sample();
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        sensors::ImuSensorPtr imu;

        std::vector<Control> controls;

        common::Time lastUpdateTime;

        boost::shared_ptr<SimWirelessReceiver> receiver;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(FinalProjectPlugin)
}