#include <arpa/inet.h>
#include <fcntl.h>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sdf/Element.hh>
#include <sys/socket.h>
#include <unistd.h>

namespace gazebo
{
    struct StatePacket
    {
        double timestamp;

        double imuAngularVelocityRPY[3];

        double imuLinearAccelerationXYZ[3];

        double imuOrientationQuat[4];

        double velocityXYZ[3];

        double positionXYZ[3];
    };

    struct MotorPacket
    {
        float motorSpeed[4] = {0.0f};
    };

    struct State
    {
        double timestamp;
        ignition::math::v4::Vector3d linearAccel;
        ignition::math::v4::Vector3d angularVel;
        ignition::math::v4::Vector3d position;
        ignition::math::v4::Quaterniond orientation;
        ignition::math::v4::Vector3d linearVel;
    };

    class ArduPilotSocket
    {
        /// \brief constructor
    public:
        ArduPilotSocket()
        {
            // initialize socket udp socket
            fd = socket(AF_INET, SOCK_DGRAM, 0);
            fcntl(fd, F_SETFD, FD_CLOEXEC);
        }

        /// \brief destructor
    public:
        ~ArduPilotSocket()
        {
            if (fd != -1)
            {
                close(fd);
                fd = -1;
            }
        }

        /// \brief Bind to an adress and port
        /// \param[in] _address Address to bind to.
        /// \param[in] _port Port to bind to.
        /// \return True on success.
    public:
        bool Bind(const char *_address, const uint16_t _port)
        {
            struct sockaddr_in sockaddr;
            this->MakeSockAddr(_address, _port, sockaddr);

            if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
            {
                ::shutdown(this->fd, 0);
                close(this->fd);
                return false;
            }
            int one = 1;
            setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<const char *>(&one), sizeof(one));

            fcntl(this->fd, F_SETFL,
                  fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
            return true;
        }

        /// \brief Connect to an adress and port
        /// \param[in] _address Address to connect to.
        /// \param[in] _port Port to connect to.
        /// \return True on success.
    public:
        bool Connect(const char *_address, const uint16_t _port)
        {
            struct sockaddr_in sockaddr;
            this->MakeSockAddr(_address, _port, sockaddr);

            if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
            {
                ::shutdown(this->fd, 0);
                close(this->fd);
                return false;
            }
            int one = 1;
            setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<const char *>(&one), sizeof(one));

            fcntl(this->fd, F_SETFL,
                  fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
            return true;
        }

        /// \brief Make a socket
        /// \param[in] _address Socket address.
        /// \param[in] _port Socket port
        /// \param[out] _sockaddr New socket address structure.
    public:
        void MakeSockAddr(const char *_address, const uint16_t _port,
                          struct sockaddr_in &_sockaddr)
        {
            memset(&_sockaddr, 0, sizeof(_sockaddr));

            _sockaddr.sin_port = htons(_port);
            _sockaddr.sin_family = AF_INET;
            _sockaddr.sin_addr.s_addr = inet_addr(_address);
        }

    public:
        ssize_t Send(const void *_buf, size_t _size)
        {
            return send(this->fd, _buf, _size, 0);
        }

        /// \brief Receive data
        /// \param[out] _buf Buffer that receives the data.
        /// \param[in] _size Size of the buffer.
        /// \param[in] _timeoutMS Milliseconds to wait for data.
    public:
        ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
        {
            fd_set fds;
            struct timeval tv;

            FD_ZERO(&fds);
            FD_SET(this->fd, &fds);

            tv.tv_sec = _timeoutMs / 1000;
            tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

            if (select(this->fd + 1, &fds, NULL, NULL, &tv) != 1)
            {
                return -1;
            }

            return recv(this->fd, _buf, _size, 0);
        }

        /// \brief Socket handle
    private:
        int fd;
    };

    class ArduPilotInterface
    {
    public:
        ArduPilotInterface(sdf::ElementPtr _sdf)
        {
            std::string fdm_addr = _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;
            std::string listen_addr = _sdf->Get("listen_addr", static_cast<std::string>("127.0.0.1")).first;
            uint32_t fdm_port_in = _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;
            uint32_t fdm_port_out = _sdf->Get("fdm_port_out", static_cast<uint32_t>(9003)).first;

            if (!socket_in.Bind(listen_addr.c_str(), fdm_port_in))
            {
                gzerr << "Failed to bind ArduPilot socket in\n";
            }

            if (!socket_out.Connect(fdm_addr.c_str(), fdm_port_out))
            {
                gzerr << "Failed to bind ArduPilot socket out\n";
            }
        }

        std::vector<float> ReceiveMotorCommands()
        {
            bool first = true;
            ssize_t recvSize = -1;
            MotorPacket pkt, temp_pkt;
            while (arduPilotOnline && (recvSize != -1 || first))
            {
                ssize_t recvSize_temp = socket_in.Recv(&temp_pkt, sizeof(MotorPacket), first ? 1000 : 0ul);
                first = false;

                if (recvSize != -1)
                {
                    pkt = temp_pkt;
                    recvSize = recvSize_temp;
                }
            }

            if (recvSize == -1)
            {
                common::Time::NSleep(100);
            }
            else
            {
                const ssize_t recvChannels = recvSize / sizeof(pkt.motorSpeed[0]);
                arduPilotOnline = true;

                std::vector<float> received;
                for (unsigned i = 0; i < recvChannels; i++)
                {
                    received.push_back(pkt.motorSpeed[i]);
                }

                return received;
            }

            return std::vector<float>();
        }

        void SendState(const State &state)
        {
            StatePacket pkt;
            pkt.timestamp = state.timestamp;

            VectorToArray(state.angularVel, pkt.imuAngularVelocityRPY);
            VectorToArray(state.linearAccel, pkt.imuLinearAccelerationXYZ);
            VectorToArray(state.linearVel, pkt.velocityXYZ);
            VectorToArray(state.position, pkt.positionXYZ);

            pkt.imuOrientationQuat[0] = state.orientation.W();
            pkt.imuOrientationQuat[1] = state.orientation.X();
            pkt.imuOrientationQuat[2] = state.orientation.Y();
            pkt.imuOrientationQuat[3] = state.orientation.Z();

            socket_out.Send(&pkt, sizeof(pkt));
        }

    private:
        void VectorToArray(ignition::math::v4::Vector3d vec, double arr[3])
        {
            arr[0] = vec.X();
            arr[1] = vec.Y();
            arr[2] = vec.Z();
        }

        ArduPilotSocket socket_in;
        ArduPilotSocket socket_out;
        bool arduPilotOnline = false;
    };
}