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
#include "./Logger.cpp"
#include "./Camera.cpp"

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
            ekf = boost::shared_ptr<EKF>(new EKF(pose.Pos().X(), pose.Pos().Y(), 0, 0));

            arduPilotInterface = boost::shared_ptr<ArduPilotInterface>(new ArduPilotInterface(_sdf));

            this->modelXYZToAirplaneXForwardZDown =
                ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
            if (_sdf->HasElement("modelXYZToAirplaneXForwardZDown"))
            {
                this->modelXYZToAirplaneXForwardZDown =
                    _sdf->Get<ignition::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
            }

            this->gazeboXYZToNED = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
            if (_sdf->HasElement("gazeboXYZToNED"))
            {
                this->gazeboXYZToNED = _sdf->Get<ignition::math::Pose3d>("gazeboXYZToNED");
            }
        }

        void LoadSensors(physics::ModelPtr model, sdf::ElementPtr _sdf)
        {
            std::string wirelessName = _sdf->Get("wirelessName", static_cast<std::string>("wireless_sensor")).first;
            std::string imuName = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
            std::string cameraName = _sdf->Get("cameraName", static_cast<std::string>("camera_sensor")).first;
            std::string cameraTopic = _sdf->Get("cameraTopic", static_cast<std::string>("camera")).first;

            std::vector<std::string> wirelessScopedName = this->model->SensorScopedName(wirelessName);
            std::vector<std::string>
                imuScopedName = this->model->SensorScopedName(imuName);
            std::vector<std::string>
                cameraScopedName = this->model->SensorScopedName(cameraName);

            if (wirelessScopedName.size() > 1)
            {
                gzwarn << "Multiple names match " << wirelessName << ". Failing.\n";
            }
            else
            {
                std::unordered_map<std::string, physics::ModelPtr> transmitters = LoadTransmitters(model, _sdf);
                this->receiver = boost::shared_ptr<SimWirelessReceiver>(new SimWirelessReceiver(model, transmitters));

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

            sensors::CameraSensorPtr cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(sensors::SensorManager::Instance()->GetSensor(cameraScopedName[0]));

            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            this->subscriber = node->Subscribe(cameraSensor->Topic(), &FinalProjectPlugin::OnCameraUpdate, this);
            this->camera = boost::shared_ptr<Camera>(new Camera(cameraSensor->ImageWidth(), cameraSensor->ImageHeight()));
        }

        std::unordered_map<std::string, physics::ModelPtr> LoadTransmitters(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            std::unordered_map<std::string, physics::ModelPtr> transmitters;

            sdf::ElementPtr transmittersSdf = sdf->GetElement("transmitter");
            while (transmittersSdf)
            {
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

            if (curTime >= lastUpdateTime)
            {
                std::vector<float> commands = arduPilotInterface->ReceiveMotorCommands();

                for (int i = 0; i < commands.size(); i++)
                {
                    controls[i].ApplyCommand(commands[i], (curTime - lastUpdateTime).Double());
                }

                std::tuple<double, double> loc = UwbLocation();
                ignition::math::Vector3d accel = BodyToNavFrame(imu->LinearAcceleration(), imu->Orientation());
                ekf->Update(std::get<0>(loc), std::get<1>(loc), accel.X(), accel.Y(), (curTime - lastUpdateTime).Double());
                EKFState ekfState = ekf->GetState();

                const ignition::math::Pose3d gazeboXYZToModelXForwardZDown =
                    this->modelXYZToAirplaneXForwardZDown +
                    ignition::math::Pose3d(ignition::math::Vector3d(ekfState.x, ekfState.y, model->WorldPose().Pos().Z()), imu->Orientation());

                const ignition::math::Pose3d NEDToModelXForwardZUp =
                    gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

                const ignition::math::Vector3d velGazeboWorldFrame =
                    ignition::math::Vector3d(ekfState.vx, ekfState.vy, model->GetLink()->WorldLinearVel().Z());
                const ignition::math::Vector3d velNEDFrame =
                    this->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);

                State state;
                state.angularVel = imu->AngularVelocity();
                state.linearAccel = imu->LinearAcceleration();
                state.linearVel = velNEDFrame;
                state.orientation = NEDToModelXForwardZUp.Rot();
                state.position = NEDToModelXForwardZUp.Pos();
                state.timestamp = model->GetWorld()->SimTime().Double();
                arduPilotInterface->SendState(state);

                lastUpdateTime = curTime;

                LogState();
            }
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

        ignition::math::Vector3d BodyToNavFrame(ignition::math::v4::Vector3d accel, ignition::math::v4::Quaterniond orientation)
        {
            ignition::math::Vector3d accelNav = ignition::math::Quaterniond(M_PI, 0, 0).RotateVector(orientation.RotateVector(accel) + ignition::math::Vector3d(0, 0, 9.8));
            return accelNav;
        }

        void LogState()
        {
            logCounter++;
            if (logCounter % 10 == 0)
            {
                EKFState ekfState = ekf->GetState();

                LogEntry entry;
                entry.estimated.x = ekfState.x;
                entry.estimated.y = ekfState.y;
                entry.estimated.vx = ekfState.vx;
                entry.estimated.vy = ekfState.vy;

                ignition::math::Vector3d pos = model->WorldPose().Pos();
                entry.truth.x = pos.X();
                entry.truth.y = pos.Y();

                ignition::math::Vector3d vel = model->GetLink()->WorldLinearVel();
                entry.truth.vx = vel.X();
                entry.truth.vy = vel.Y();

                logger.WriteRecord(entry);
            }
        }

        void OnCameraUpdate(ConstImageStampedPtr &msg)
        {
            common::Time curTime = model->GetWorld()->SimTime();
            if (msg && curTime >= lastCameraTime + 0.5)
            {
                std::tuple<double, double, double> *position = this->camera->ProcessImage(msg->image().data().c_str());
                if (position != nullptr)
                {
                    gzmsg << std::get<2>(*position) << "\n" << std::get<0>(*position) << " " << std::get<1>(*position) << "\n"
                          << model->WorldPose().Pos().X() << " " << model->WorldPose().Pos().Y() << "\n\n";
                }
                lastCameraTime = curTime;
            }
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        sensors::ImuSensorPtr imu;
        sensors::CameraSensorPtr cameraSensor;

        transport::NodePtr node;
        transport::SubscriberPtr subscriber;
        boost::shared_ptr<Camera> camera;

        std::vector<Control> controls;

        common::Time lastUpdateTime;
        common::Time lastCameraTime;

        boost::shared_ptr<SimWirelessReceiver> receiver;
        boost::shared_ptr<std::unordered_map<std::string, ignition::math::v4::Vector3d>> transmitterPositions;

        boost::shared_ptr<EKF> ekf;
        double dt = 0.1;

        boost::shared_ptr<ArduPilotInterface> arduPilotInterface;

        std::mutex mutex;

        ignition::math::Pose3d modelXYZToAirplaneXForwardZDown;
        ignition::math::Pose3d gazeboXYZToNED;

        Logger logger;
        int logCounter = 0;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(FinalProjectPlugin)
}