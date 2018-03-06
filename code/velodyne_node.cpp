#include "time_util.h"
#include "libgen.h"
#include <string>
#include <cstring>
#include <VelodyneMonitor.h>
#include <iostream>
#include <atomic>
#include <chrono>
#include <math.h>

using namespace velodyne;
using namespace std;

bool need_reset = false;
VelodyneMonitor * monitor = NULL;

unsigned timestamp_error_threshold = 0;
unsigned timestamp_warn_threshold = 0;
int fixCharacterPosition = 17;
double gps_diff_threshold = 0.0;
double gps_fix_lost_max_duration = 2.0;
double max_time_between_datagrams = 0.0;

timespec previous_time, last_timeout;
bool have_previous_timestamp = false;

// The C++ standard requires an unsigned long to be capable
// of the range 0 to 4,294,967,295. Since the maximum LiDAR
// four byte timestamp is 3,599,999,999 microseconds, the
// data type is sufficient.
unsigned long expectedLidarTimestamp = 0UL;
unsigned long currentLidarTimestamp = 0UL;

const uint32_t ONE_HOUR = 3600000000U;
const unsigned long ONE_HOUR_IN_MILLIS = 3600000UL;
const unsigned long MICROS_PER_MILLI = 1000UL;

atomic<unsigned int> successiveTimeouts;
unsigned int tooManyTimeouts = 5;

unsigned velodyne_packets_index = 0;

timespec prevous_time, current_time;
bool have_previous_time = false;

bool synced_with_gps = false;
unsigned sync_warning_message_count = 0;

unsigned data_packet_count = 0;
unsigned position_packet_count = 0;
unsigned valid_gprmc_count = 0;
unsigned valid_pps_count = 0;

const unsigned data_packets_per_second_threshold = 1500;
const unsigned gprmc_count_per_second_threshold = 150;
const unsigned pps_count_per_second_threshold = 150;

struct velodyne_status_struct
{
    bool data_packets_good;
    bool gprmc_good;
    bool pps_good;
};

velodyne_status_struct velodyne_status;

void socketErrorHandler(SocketError &e)
{
    cout << "Socket error while communicating with velodyne unit" << endl;
    need_reset = true;
}

void timeoutHandler(Timeout & e)
{
    /* This function is called when the underlying library
       function doesn't find an incoming LiDAR datagram
       available over the network within the amount of time
       which is hard-coded within the library. This doesn't
       confirm that there's an actual error, only that
       the next datagram didn't become available within the
       strict timeframe it is supposed to become available.

       The parameter tooManyTimeouts allows a small number
       of timeouts to occur, without ill effect. */

    timespec now, diff;
    double time_diff_double;    
    ++successiveTimeouts;
    clock_gettime(CLOCK_REALTIME, &now);
    timeval_subtract(&diff, &now, &last_timeout);
    time_diff_double = (double)diff.tv_sec + (double)diff.tv_nsec/1.0e9;

    if (time_diff_double > 1.0) {
        cout << successiveTimeouts << " successive timeouts waiting for next LiDAR datagram" << endl;
        last_timeout = now;
        if (successiveTimeouts >= tooManyTimeouts) {
            need_reset = true;
        }
    }
}


bool initVelodyneMonitor(struct in_addr addr)
{
    bool result = true;
    try {
        monitor = new VelodyneMonitor(addr, socketErrorHandler, timeoutHandler);
        monitor->start();
        have_previous_timestamp = false;
    } catch (SocketError & e) {
        cout << "Socket error while initializing Velodyne monitor. " << "Deleting the VelodyneMonitor object." << endl;
        delete monitor;
        return false;
    } catch (Timeout & e) {
        cout << "Timeout while initializing Velodyne monitor" << "Deleting the VelodyneMonitor object." << endl;
        delete monitor;
        return false;
    }

    return result;
}

bool isFinalLaserPacket(const LaserPacket & item)
{
    return item.laser_bank_measures[0].start_identifier == 0;
}

bool isFinalPositionPacket(const PositioningPacket & item)
{
    // An item like this should be a constant value unless the datagram was explicitly cleared.
    return item.unused1[0] == 1;
}

bool gpsIsSynced(const PositioningPacket & datagram)
{
    // 0123456789ABCDE
    // $GPRMC,HHMMSS,A ... on the Garmin
    // $GPRMC,HHMMSS.ss,A ... on the Novatel
    auto message = datagram.nmea_sentence;

    return message[0] == '$' && message[1] == 'G' && message[2] == 'P' &&
        message[3] == 'R' && message[4] == 'M' && message[5] == 'C' &&
        message[fixCharacterPosition] == 'A';
}


bool retrieveLidarInfo(const char * path)
{
    string return_value;
    FILE * fp;
    char result[0x100];

//    string full_path = string(path) + "/home/earthmine/Vishi/real_time_lidar/get_lidar_info.py";
    string full_path = "/home/earthmine/Vishi/real_time_lidar/get_lidar_info.py";
    cout << "Full Path: " << full_path << endl;
    fp = popen(full_path.c_str(), "r");
    if (fp == NULL) {
        cout << "Missing get_lidar_info.py; expect missing data from metadata files." << endl;
        return false;
    }

    string serial;
    string firmware;
    string model;

    bool ret_val = true;

    char * fgets_result = fgets(result, sizeof result - 1, fp);
    if (fgets_result != NULL) {
        serial = fgets_result;
        fgets_result = fgets(result, sizeof result - 1, fp);
    }
    if (fgets_result != NULL) {
        firmware = fgets_result;
        fgets_result = fgets(result, sizeof result - 1, fp);
    }
    if (fgets_result != NULL) {
        model = fgets_result;
    } else {
        cout << "Error reading serial/firmware from lidar. Expect nonsense in corresponding metadata." << endl;
        ret_val = false;
    }

    pclose(fp);

    return ret_val;
}


int main(int argc, char * argv[])
{
    // For whatever reason, the lidar is sometimes not ready to talk to us, so
    // I'm putting this call into a retry loop.
    unsigned retries = 4;
    unsigned timeout = 1;

    // Copy argv[0] to avoid dirname mangling it.
    size_t command_path_length = strlen(argv[0]);
    char command_path[command_path_length + 1];
    strncpy(command_path, argv[0], command_path_length);
    command_path[command_path_length] = '\0';
    string command_path_dir = dirname(command_path);

/*    bool got_lidar_info = retrieveLidarInfo(command_path_dir.c_str());
    while (retries != 0 && ! got_lidar_info) {
        sleep(timeout);
        timeout *= 2;
        --retries;
        got_lidar_info = retrieveLidarInfo(command_path_dir.c_str());
    }
*/
    fixCharacterPosition = 17;

    max_time_between_datagrams = 1.0;

    timestamp_error_threshold = 2000000;

    timestamp_warn_threshold = 1;

    string ip_address = "192.168.1.201";

    struct in_addr addr; 
    if (inet_aton(ip_address.c_str(), &addr) == 0) {
        cout << ip_address << " is not a valid IP address." << endl;
        exit(1);
    }

    bool laser_queue_end = false;
    bool position_queue_end = false;
    LaserPacket datagram;
    successiveTimeouts = 0;
    timespec last_status_time, current_time, lost_sync_time;
    clock_gettime(CLOCK_REALTIME, &last_status_time);

    while (! initVelodyneMonitor(addr)) {
        msdelay(2000);
    }

    while (! need_reset) {
        try {
            datagram = monitor->getLaserQueue().pop(chrono::milliseconds(100));
            ++data_packet_count;
            if (remainder(data_packet_count, 100))
            {
                cout << (double)datagram.laser_bank_measures[0].angle_hundredths/100.0 << endl;
            }

            if (data_packet_count > 200)
                exit(1);


            clock_gettime(CLOCK_REALTIME, &current_time);

            if (! monitor->getPositionQueue().empty()) {
                auto position_datagram = monitor->getPositionQueue().pop();
                ++position_packet_count;
                switch (position_datagram.pps_status) {
                    case 0:
                        cout << "Velodyne scanner cannot see a pps signal" << endl;
                        break;
                    case 1:
                        ++valid_pps_count;
                        cout << "Velodyne scanner sees an intermittent pps signal" << endl;
                        break;
                    case 2:
                        ++valid_pps_count;
                        break; // This is the good case. Nothing to do.
                    case 3:
                        cout << "Velodyne scanner sees a thrashing pps signal" << endl;
                        break;
                    default:
                        // Assume this is an older Velodyne unit and don't complain.
                        ++valid_pps_count;
                }
                const bool gps_synced = gpsIsSynced(position_datagram);
                static bool waiting_for_resync = false;

                if (gps_synced) {
                    ++valid_gprmc_count;
                }
                if (isFinalPositionPacket(position_datagram)) {
                    position_queue_end = true;
                } else if (gps_synced) {
                    synced_with_gps = true;
                    waiting_for_resync = false;
                    sync_warning_message_count = 0;
                } else if (synced_with_gps) {
                    // Lost GPS fix.
                    if (waiting_for_resync) {
                        timespec diff, now;
                        double diff_double;
                        clock_gettime(CLOCK_REALTIME,&now);

                        timeval_subtract(&diff, &now, &lost_sync_time);
                        diff_double = (double)diff.tv_sec + (double)diff.tv_nsec/1.0e9;

                        if (diff_double > gps_fix_lost_max_duration) {
                            cout << "Lost GPS fix for too long. Aborting." << endl;
                        }
                    } else {
                        waiting_for_resync = true;
                        clock_gettime(CLOCK_REALTIME, &lost_sync_time);
                    }
                } else {
                    // The GPS has not yet reported a fix.
                    cout << "LiDAR is waiting for the GPS to report a fix. DataPacketCount: " << data_packet_count << endl;
                }
            }

            if (isFinalLaserPacket(datagram)) {
                laser_queue_end = true; // Should only happen if there is an error.
            } else {
                successiveTimeouts = 0;

                currentLidarTimestamp = datagram.gps_timestamp;

                // Do the various time checks to see if this datagram's timing is sane.
                // Start by comparing the ros time to previous ros time.
                if (have_previous_time) {
                    timespec time_diff;
                    double time_diff_double;
                    timeval_subtract(&time_diff, &current_time, &previous_time);
                    time_diff_double = double(time_diff.tv_sec) + double(time_diff.tv_nsec)/1.0e9;

                    if (time_diff_double > max_time_between_datagrams) {
                        cout << "Excessive time between receipt of LiDAR datagrams." << endl;
                    }
                }

                if (have_previous_timestamp) {
                    // Always make the difference positive, and if it's greater than half an hour, consider
                    // it to be negative:
                    uint64_t b = currentLidarTimestamp;
                    uint64_t a = expectedLidarTimestamp;
                    b += ONE_HOUR - a;
                    a = ONE_HOUR;
                    if (b >= ONE_HOUR) {
                        b -= ONE_HOUR;
                    }
                    // After this point, b is guaranteed less than ONE_HOUR. Use its position on the dial
                    // to decide whether it's greater or less than b.
                    unsigned timestamp_diff;
                    // In this case, b is considered clockwise of a, otherwise counter.
                    if (b < ONE_HOUR / 2) {
                        timestamp_diff = currentLidarTimestamp - expectedLidarTimestamp;
                    } else {
                        timestamp_diff = (currentLidarTimestamp + ONE_HOUR) - expectedLidarTimestamp;
                    }

                    if (timestamp_diff > (ONE_HOUR / 2)) {
                        // Negative timestamp diff
                        cout << "Current LiDAR timestamp before previous." << " currentLidarTimestamp: " <<
                            currentLidarTimestamp << " expectedLidarTimestamp: " << expectedLidarTimestamp << endl;
                    } else if (timestamp_diff > timestamp_error_threshold) {
                        cout <<"Current LiDAR timestamp too far forward of expected. " <<
                            " timestamp_diff in microsec: " << timestamp_diff << ". Aborting session." << endl;
                    } else if (timestamp_diff > timestamp_warn_threshold) {
                        cout << "Current LiDAR timestamp further forward than expected." <<
                            " timestamp_diff in microsec: " << timestamp_diff << endl;
                    }
                }
            }

            // Update the various time values.
            expectedLidarTimestamp = (currentLidarTimestamp + 551) % ONE_HOUR;
            have_previous_timestamp = true;
            previous_time = current_time;
            have_previous_time = true;


        } catch (VelodyneMonitor::LaserPacketQueue::Timeout & e) {
            // Nothing to do. Just allows ros to exit.
        }

        timespec now;
        clock_gettime(CLOCK_REALTIME, &now);

        double status_time_diff_double;
        timespec status_time_diff;

        timeval_subtract(&status_time_diff, &now, &last_status_time);

        status_time_diff_double = double(status_time_diff.tv_sec) + double(status_time_diff.tv_nsec)/1.0e9;

        if (status_time_diff_double  >= 1.0) {
            last_status_time = now;

            if (data_packet_count > data_packets_per_second_threshold) {
                velodyne_status.data_packets_good = true;
            } else {
                velodyne_status.data_packets_good = false;
            }

            if (valid_gprmc_count > gprmc_count_per_second_threshold) {
                velodyne_status.gprmc_good = true;
            } else {
                velodyne_status.gprmc_good = false;
            }

            if (valid_pps_count > pps_count_per_second_threshold) {
                velodyne_status.pps_good = true;
            } else {
                velodyne_status.pps_good = false;
            }

            data_packet_count = 0;
            position_packet_count = 0;
            valid_gprmc_count = 0;
            valid_pps_count = 0;
        }
    }

    if (need_reset) {
        cout << "Resetting the LiDAR monitor" << endl;
        monitor->stop(); // There should be at least a poison element after this returns.

        while (!laser_queue_end) {
            auto datagram = monitor->getLaserQueue().pop();
            if (isFinalLaserPacket(datagram)) {
                laser_queue_end = true;
            }
        }

        while (!position_queue_end) {
            auto datagram = monitor->getPositionQueue().pop();
            if (isFinalPositionPacket(datagram)) {
                position_queue_end = true;
            }
        }

        delete monitor;
        successiveTimeouts = 0;
        need_reset = false;
        laser_queue_end = false;
        position_queue_end = false;
        synced_with_gps = false;
        velodyne_packets_index = 0;
    }   


    if (monitor) {
        monitor->stop(); // This automatically makes both queues non-empty.

        while (!laser_queue_end) {
            cout << "Clearing LiDAR datagram queue." << endl;
            auto datagram = monitor->getLaserQueue().pop();
            if (datagram.laser_bank_measures[0].start_identifier == 0) {
                laser_queue_end = true;
            }
        }

        while (!position_queue_end) {
            cout << "Clearing LiDAR position datagram queue." << endl;
            auto datagram = monitor->getPositionQueue().pop();
            if (isFinalPositionPacket(datagram)) {
                position_queue_end = true;
            }
        }

        delete monitor;
    }

}
