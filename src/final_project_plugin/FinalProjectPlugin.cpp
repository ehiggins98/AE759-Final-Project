#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Subscriber.hh>
#include "./Motors.cpp"

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

            LoadSensors(_sdf);

            LoadControl(_parent, _sdf->GetElement("control"));
        }

        void LoadSensors(sdf::ElementPtr _sdf)
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
                this->wireless = std::dynamic_pointer_cast<sensors::WirelessReceiver>(sensors::SensorManager::Instance()->GetSensor(wirelessScopedName[0]));
                transport::NodePtr node(new transport::Node());
                node->Init();

                this->subscriber = node->Subscribe(this->wireless->Topic(), &FinalProjectPlugin::ReceiveWirelessData, this);
                this->wirelessNode = node;
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
            common::Time curTime = model->GetWorld()->SimTime();
            if (curTime > lastUpdateTime)
            {
                for (std::vector<Control>::iterator itr = controls.begin(), end = controls.end(); itr != end; itr++)
                {
                    itr->ApplyCommand(1, (curTime - lastUpdateTime).Double());
                }
            }

            lastUpdateTime = curTime;
        }

        void ReceiveWirelessData(ConstWirelessNodesPtr &_msg)
        {
            gzmsg << _msg->node_size() << "\n";
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        sensors::ImuSensorPtr imu;
        sensors::WirelessReceiverPtr wireless;
        transport::NodePtr wirelessNode;
        transport::SubscriberPtr subscriber;

        std::vector<Control> controls;

        common::Time lastUpdateTime;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(FinalProjectPlugin)
}