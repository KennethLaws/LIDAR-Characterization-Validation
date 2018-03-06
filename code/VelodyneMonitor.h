#if !defined __BAH_VELODYNEMONITOR_H_
#define __BAH_VELODYNEMONITOR_H_

#include "BlockingQueue.h"
#include <thread>
#include <atomic>

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>

// This class differs from other monitors in that there isn't any initialization or communication
// required, so therefore no underlying device abstraction.

namespace velodyne
{

struct SingleLaserMeasure {
    uint16_t range;
    uint8_t intensity;
} __attribute__((packed));

struct LaserBankMeasure {
    uint16_t start_identifier;
    uint16_t angle_hundredths;
    SingleLaserMeasure laser_measures[32];
} __attribute__((packed));

struct LaserPacket {
    LaserBankMeasure laser_bank_measures[12];
    uint32_t gps_timestamp;
    uint8_t unused1;
    uint8_t unused2;
} __attribute__((packed));

// All the measurements are significant to only 12 bits. The 4 unsignificant are used as an index, and quite
// pointless for programmatic interpretation.
// Multiply gyro value by 0.09766 to get degrees/sec.
// Multiply temperature values by .01453 to get Celsius.
// Multiply accelerometer values by 0.001221 to get "G"s.
// Not sure why there is so much empty space in this packet. Shrug.
struct PositioningPacket {
    uint8_t     unused1[14];
    unsigned    gyro1_index     : 4;
    int         gyro1           : 12;
    unsigned    temp1_index     : 4;
    int         temp1           : 12;
    unsigned    x_accel1_index  : 4;
    int         x_accel1        : 12;
    unsigned    y_accel1_index  : 4;
    int         y_accel1        : 12;
    unsigned    gyro2_index     : 4;
    int         gyro2           : 12;
    unsigned    temp2_index     : 4;
    int         temp2           : 12;
    unsigned    x_accel2_index  : 4;
    int         x_accel2        : 12;
    unsigned    y_accel2_index  : 4;
    int         y_accel2        : 12;
    unsigned    gyro3_index     : 4;
    int         gyro3           : 12;
    unsigned    temp3_index     : 4;
    int         temp3           : 12;
    unsigned    x_accel3_index  : 4;
    int         x_accel3        : 12;
    unsigned    y_accel3_index  : 4;
    int         y_accel3        : 12;
    uint8_t     unused2[160];
    uint32_t    gps_timestamp;
    uint8_t     pps_status;
    uint8_t     unused3[3];
    char        nmea_sentence[72];
    uint8_t     unused4[234];
} __attribute__((packed));


struct SocketError {
    SocketError(int e) : error(e) { }
    int error; // errno error
};

struct Timeout {
};


class VelodyneMonitor
{
public: // Typedefs
    typedef void (*SocketErrorHandler)(SocketError &);
    typedef void (*TimeoutHandler)(Timeout &);
    typedef BlockingQueue<LaserPacket, 0> LaserPacketQueue; // Zero signifies indefinite queue length.
    typedef BlockingQueue<PositioningPacket, 0> PositionQueue;

public: // Construction, etc.
    // struct in_addr is an ip address; not large enough to keep off the stack.
    VelodyneMonitor(struct in_addr, SocketErrorHandler, TimeoutHandler);
    ~VelodyneMonitor();

    LaserPacketQueue & getLaserQueue() {
        return _laser_packet_queue;
    }
    PositionQueue & getPositionQueue() {
        return _position_queue;
    }

    void start();
    void stop();

private: // Thread main method.
    void run();

private:
    // Socket Stuff
    struct in_addr _ip_address;
    int _laser_packet_socket;
    int _position_socket;

    // Queue Stuff
    LaserPacketQueue _laser_packet_queue;
    PositionQueue _position_queue;

    // Thread and locking
    std::thread * _thread;
    std::atomic<bool> _terminate;

    // Error handler callbacks
    SocketErrorHandler _socket_error_handler;
    TimeoutHandler _timeout_handler;
};

}; // namespace velodyne

#endif
