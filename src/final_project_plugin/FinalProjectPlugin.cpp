#include <math.h>
#include <mutex>

#include <Eigen/Dense>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>

#include "./Motors.cpp"
#include "./SimWirelessReceiver.cpp"
#include "./EKF.cpp"
#include "./ArduPilotInterface.cpp"

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

            ignition::math::v4::Pose3d pose = model->WorldPose();
            ekf = boost::shared_ptr<EKF>(new EKF(pose.Pos().X(), pose.Pos().Y(), 0, 0, dt));

            arduPilotInterface = boost::shared_ptr<ArduPilotInterface>(new ArduPilotInterface(_sdf));
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

                this->transmitterPositions = boost::shared_ptr<std::unordered_map<std::string, ignition::math::v4::Vector3d>>(new std::unordered_map<std::string, ignition::math::v4::Vector3d>());
                for (std::unordered_map<std::string, physics::ModelPtr>::const_iterator itr = transmitters.cbegin(), end = transmitters.cend(); itr != end; itr++)
                {
                    this->transmitterPositions->insert(std::pair<std::string, ignition::math::v4::Vector3d>(itr->first, itr->second->WorldPose().Pos()));
                }
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
            std::lock_guard<std::mutex> lock(mutex);
            common::Time curTime = model->GetWorld()->SimTime();

            if (curTime > lastUpdateTime)
            {
                std::vector<float> commands = arduPilotInterface->ReceiveMotorCommands();
                for (int i = 0; i < controls.size(); i++)
                {
                    controls[i].ApplyCommand(commands[i], (curTime - lastUpdateTime).Double());
                }

                std::tuple<double, double> loc = UwbLocation();
                std::tuple<double, double, double> accel = BodyToNavFrame(imu->LinearAcceleration(), imu->Orientation());
                ekf->Update(std::get<0>(loc), std::get<1>(loc), std::get<0>(accel), std::get<1>(accel));
                EKFState ekfState = ekf->GetState();

                ignition::math::Vector3d vel = model->GetLink()->WorldLinearVel();

                State state;
                state.angularVel = imu->AngularVelocity();
                state.linearAccel = imu->LinearAcceleration();
                state.linearVel = ignition::math::Vector3d(ekfState.vx, ekfState.vy, vel.Z() * -1);
                state.orientation = imu->Orientation();
                state.position = ignition::math::Vector3d(ekfState.x, ekfState.y, model->WorldPose().Pos().Z());
                state.timestamp = model->GetWorld()->SimTime().Double();

                arduPilotInterface->SendState(state);
            }

            lastUpdateTime = curTime;
        }

        std::tuple<double, double> UwbLocation()
        {
            std::unordered_map<std::string, double> distances = receiver->Sample();

            std::vector<double> distanceList;
            std::vector<ignition::math::v4::Vector3d> posList;

            for (std::unordered_map<std::string, double>::const_iterator itr = distances.cbegin(), end = distances.cend(); itr != end; itr++)
            {
                distanceList.push_back(itr->second);
                posList.push_back(transmitterPositions->find(itr->first)->second);
            }

            Eigen::MatrixXd H(distanceList.size() - 1, 2);
            Eigen::VectorXd z(distanceList.size() - 1);

            for (int i = 1; i < distanceList.size(); i++)
            {
                H(i - 1, 0) = 2 * posList[0].X() - 2 * posList[i].X();
                H(i - 1, 1) = 2 * posList[0].Y() - 2 * posList[i].Y();
                z(i - 1) = pow(distanceList[i], 2) - pow(distanceList[0], 2) + pow(posList[0].X(), 2) - pow(posList[i].X(), 2) + pow(posList[0].Y(), 2) - pow(posList[i].Y(), 2);
            }

            Eigen::Vector2d uavPos = (H.transpose() * H).inverse() * H.transpose() * z;
            return std::tuple<double, double>(uavPos(0), uavPos(1));
        }

        std::tuple<double, double, double> BodyToNavFrame(ignition::math::v4::Vector3d accel, ignition::math::v4::Quaterniond orientation)
        {
            double phi = orientation.Roll();
            double theta = orientation.Pitch();
            double psi = orientation.Yaw();

            Eigen::Matrix3d R;
            R << cos(theta) * cos(psi),
                sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi),
                sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi),
                cos(theta) * sin(psi),
                cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi),
                cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi),
                -sin(theta),
                sin(phi) * cos(theta),
                cos(phi) * cos(theta);

            Eigen::Vector3d accelVec;
            accelVec << accel.X(), accel.Y(), accel.Z();

            Eigen::Vector3d g;
            g << 0, 0, -g;

            Eigen::Vector3d accelNav = R * accelVec + g;
            return std::tuple<double, double, double>(accelNav(0), accelNav(1), accelNav(2));
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        sensors::ImuSensorPtr imu;

        std::vector<Control> controls;

        common::Time lastUpdateTime;

        boost::shared_ptr<SimWirelessReceiver> receiver;
        boost::shared_ptr<std::unordered_map<std::string, ignition::math::v4::Vector3d>> transmitterPositions;

        boost::shared_ptr<EKF> ekf;
        double dt = 0.1;

        boost::shared_ptr<ArduPilotInterface> arduPilotInterface;

        std::mutex mutex;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(FinalProjectPlugin)
}