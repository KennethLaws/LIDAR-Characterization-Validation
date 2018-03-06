/* This implementation of lidar testing has been made independent of Lib_Camera_Control to avoid issues with
 * GPS syncing as that is not a requirement for lidar calibration testing on the bench. Lib_Camera_Control
 * code has been modified to remove GPS dependencies and is included in this repo. A reference file named
 * reference_point_set.ascii is required to be present in the current directory for doing a comparison test.
 * If the user wants to collect a reference dataset with a lidar known to have good calibration, this program
 * can collect a reference dataset based on user entry.

 * 11/03/2017 Kenneth Laws - Added saving an intensity reference scan.
 * The intensity scan will be saved to an ascii text file named reference_intensity_set.ascii
 */



#include "time_util.h"
#include "libgen.h"
#include <string>
#include <cstring>
#include <VelodyneMonitor.h>
#include <iostream>
#include <atomic>
#include <chrono>
#include <math.h>
#include <fstream>
#include <sstream>

// program options
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/tokenizer.hpp> 

#include "gicp.h"

using namespace velodyne;
using namespace std;
using namespace dgc::gicp;
namespace po = boost::program_options;

bool laser_queue_end = false;
bool position_queue_end = false;

int num_pts_collected = 0;
bool pts_invalid_warning_done = false;
bool populate_ref_dataset = false;
bool need_reset = false;
bool scan_collected = false;
VelodyneMonitor * monitor = NULL;

unsigned timestamp_error_threshold = 0;
unsigned timestamp_warn_threshold = 0;
int fixCharacterPosition = 17;
double gps_diff_threshold = 0.0;
double gps_fix_lost_max_duration = 2.0;
double max_time_between_datagrams = 0.0;

double prev_rotation_angle = 0;

timespec previous_time, last_timeout;
bool have_previous_timestamp = false;

/* HDL32 laser angles */
double laser_vertical_angles[32] = {-30.67, -9.33, -29.33, -8.00, -28.00, -6.66, -26.66, -5.33,-25.33,-4.00,
  -24.00,-2.67,-22.67,-1.33,-21.33,0.00,-20.00,1.33,-18.67,2.67,-17.33,4.00,-16.00,5.33,-14.67,6.67,-13.33,
  8.00,-12.00,9.33,-10.67,10.67};

string command_path_dir;
ofstream ref_data_file;
ofstream ref_intnsty_file;
GICPPointSet p1, p2;

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

unsigned data_packet_count = 0;
unsigned position_packet_count = 0;

const unsigned data_packets_per_second_threshold = 1500;

struct velodyne_status_struct
{
  bool data_packets_good;
};

velodyne_status_struct velodyne_status;

static bool debug = false;
static double gicp_epsilon = 1e-3;
static double max_distance = 5.;

bool load_points(GICPPointSet *set, const char* ptfilename, const char* intstyfilename) {
  bool error = false;

  ifstream inpt(ptfilename);
  if(!inpt) {
    cout << "Could not open '" << ptfilename << "'." << endl;
    return true;
  }

  // open the intensity reference file
  ifstream inI(intstyfilename);
  if(!inI) {
    cout << "Could not open '" << intstyfilename << "'." << endl;
    return true;
  }


  string lineP, lineI;
  GICPPoint pt;

  pt.range = -1;
  for(int k = 0; k < 3; k++) {
    for(int l = 0; l < 3; l++) {
      pt.C[k][l] = (k == l)?1:0;
    }
  }    
  while(getline(inpt, lineP) && getline(inI, lineI)) {
    istringstream sinP(lineP);
    sinP >> pt.x >> pt.y >> pt.z;    
    istringstream sinI(lineI);
    sinI >> pt.intensity;    
    if (pt.x != 0 && pt.y != 0 && pt.z != 0)
    {
      set->AppendPoint(pt);    
    }
  }
  inpt.close();
  inI.close();

  return false;  
}


/* functions to communicate with lidar start */

void socketErrorHandler(SocketError &e)
{
  cout << "Socket error while communicating with velodyne unit" << endl;
  need_reset = true;
}

void timeoutHandler(Timeout & e)
{
  /* This function is called when no LiDAR datagram is
     available over the network within the amount of time
     which is hard-coded. This doesn't
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

/* lidar info query */

bool retrieveLidarInfo(const char * path)
{
  string return_value;
  FILE * fp;
  char result[0x100];

  //    string full_path = string(path) + "/home/earthmine/Vishi/real_time_lidar/get_lidar_info.py";
  string full_path = "./get_lidar_info.py";
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

  cout << "Lidar Serial: " << serial << " Firmware: " << firmware << " Model: " << model << endl;

  pclose(fp);

  return ret_val;
}

/* function to collect either reference data file or a scan from lidar being tested depending on user entry */

bool collect_scan()
{

  string ip_address = "192.168.1.201";
  unsigned retries = 4;
  unsigned timeout = 1;

  bool got_lidar_info = retrieveLidarInfo(command_path_dir.c_str());

  // For whatever reason, the lidar is sometimes not ready to talk to us, so
  // I'm putting this call into a retry loop.
  while (retries != 0 && ! got_lidar_info) {
    sleep(timeout);
    timeout *= 2;
    --retries;
    got_lidar_info = retrieveLidarInfo(command_path_dir.c_str());
  }


  cout << " Scanning lidar points ... " << endl; 

  struct in_addr addr; 
  if (inet_aton(ip_address.c_str(), &addr) == 0) {
    cout << ip_address << " is not a valid IP address." << endl;
    exit(1);
  }

  LaserPacket datagram;
  successiveTimeouts = 0;
  timespec last_status_time, current_time, lost_sync_time;
  clock_gettime(CLOCK_REALTIME, &last_status_time);

  while (! initVelodyneMonitor(addr)) {
    msdelay(2000);
  }

  /* collect lidar data */
  while (! scan_collected ) {
    while (! need_reset && ! scan_collected) {
      double rotation_angle, x, y, xy, z, range, inten;
      try {
        datagram = monitor->getLaserQueue().pop(chrono::milliseconds(100));            
        ++data_packet_count;

        for (int i = 0; i < 12; i++)
        {
          rotation_angle = (double)datagram.laser_bank_measures[i].angle_hundredths/100.0;

          for (int j = 0; j < 32; j++)
          {   
            range = datagram.laser_bank_measures[i].laser_measures[j].range * 0.002;
            inten = datagram.laser_bank_measures[i].laser_measures[j].intensity;                   

            z = range*sin(laser_vertical_angles[j]*M_PI/180.0);
            xy = range*cos(laser_vertical_angles[j]*M_PI/180.0);

            x = xy*sin(rotation_angle*M_PI/180.0);
            y = xy*cos(rotation_angle*M_PI/180.0);

            if (range > 2)
            {
              if (populate_ref_dataset)
              {
                ref_data_file << x << " " << y << " " << z << endl;
                ref_intnsty_file << j+1 << " " << rotation_angle << " " << range << " " << inten << endl;
                num_pts_collected++;
              }
              else
              {

                GICPPoint pt;
                pt.range = range;
                pt.intensity = inten;
                pt.x = x;
                pt.y = y;
                pt.z = z;

                p2.AppendPoint(pt);
                num_pts_collected++;
              }
            }

          }                

        }

        /* condition for detecting a full rotation.*/
        if (prev_rotation_angle > 355 && rotation_angle < 5 && data_packet_count > 200) 
        {
          /* for HDL-32 8000+ points is reasonable for a comparison scan */
          if (num_pts_collected > 8000)
          {
            cout << "Done collecting scan. " << endl;
            scan_collected = true;
            ref_data_file.close();
            ref_intnsty_file.close();
          }
          else
          {
            if (data_packet_count > 600 && !pts_invalid_warning_done)
            {
              cout << "Not many valid datapoints being collected. Is Lidar too close to the objects being scanned?" << endl;
              pts_invalid_warning_done = true;
            }
          }
        }

        prev_rotation_angle = rotation_angle;       

        clock_gettime(CLOCK_REALTIME, &current_time);

        if (! monitor->getPositionQueue().empty()) {
          auto position_datagram = monitor->getPositionQueue().pop();
          ++position_packet_count;

          if (isFinalPositionPacket(position_datagram)) {
            position_queue_end = true;
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

        data_packet_count = 0;
        position_packet_count = 0;
      }
    }

    /* in case reset is needed */
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
      velodyne_packets_index = 0;
    }
  }

}

/*  functions to communicate with lidar end */

int main(int argc, char * argv[])
{
  fixCharacterPosition = 17;

  max_time_between_datagrams = 1.0;

  timestamp_error_threshold = 2000000;

  timestamp_warn_threshold = 3;
  int user_entry = 0;


  string reference_dataset = "reference_point_set.ascii";        
  string intensity_dataset = "reference_intensity_set.ascii";        

  // Copy argv[0] to avoid dirname mangling it.
  size_t command_path_length = strlen(argv[0]);
  char command_path[command_path_length + 1];
  strncpy(command_path, argv[0], command_path_length);
  command_path[command_path_length] = '\0';
  command_path_dir = dirname(command_path);

  double rx,ry,rz,tx,ty,tz;    

  dgc_transform_t t_base, t0, t1, t_result;

  /* ask user for entry */
  cout << "To collect reference dataset with a known good lidar, enter 1. Any other key to test a lidar" <<
    " against the reference dataset." << endl;

  cin >> user_entry;

  if (user_entry == 1)
  {
    populate_ref_dataset = true;
  }

  /* load reference dataset if available */
  if (!populate_ref_dataset)
  {
    bool error = false;
    error = load_points(&p1, reference_dataset.c_str(), intensity_dataset.c_str());
    if(error) {
      cout << "No file named: " << reference_dataset <<
        " exists. Please provide a reference file or find" << 
        " a known good lidar to generate one using this program. "
        << endl;
      return 1;
    }
    cout << "Loaded " << reference_dataset << ": " << p1.Size() << " points into GICPPointSet 1." << endl;

   

  }
  else
  {
    ref_data_file.open(reference_dataset);
    ref_intnsty_file.open(intensity_dataset);
  }

  /* collect a scan or populate reference_dataset based on user entry */
  collect_scan();

  if (scan_collected && !populate_ref_dataset)
  {
    dgc_transform_identity(t_base);
    dgc_transform_identity(t0);
    dgc_transform_identity(t1);

    cout << "Loaded " << p2.Size() << " points into GICPPointSet 2." << endl;  

    // build kdtrees and normal matrices
    cout << "Building KDTree and computing surface normals/matrices..." << endl;

    /* build kdtrees and compute matrices for ICP */
    p1.SetGICPEpsilon(gicp_epsilon);
    p2.SetGICPEpsilon(gicp_epsilon);  
    p1.BuildKDTree();
    p1.ComputeMatrices();
    p2.BuildKDTree();
    p2.ComputeMatrices();

    // align the point clouds
    cout << "Aligning point cloud..." << endl;
    dgc_transform_copy(t1, t0);
    p2.SetDebug(debug);
    p2.SetMaxIterationInner(8);
    p2.SetMaxIteration(100);
    int iterations = p2.AlignScan(&p1, t_base, t1, max_distance);

    // print the result
    if (iterations < 99)
    {
      cout << "Scans Converged in : " << iterations << " iterations" << endl;
      dgc_transform_print(t_base, "t_base");
      dgc_transform_print(t0, "t0");  
      dgc_transform_print(t1, "t1");

      dgc_transform_get_translation(t1, &tx, &ty, &tz);
      dgc_transform_get_rotation(t1, &rx, &ry, &rz);

      cout << "rx: " << rx*180.0/M_PI << " ry: " << ry*180.0/M_PI << " rz: " << rz*180.0/M_PI << endl;
      cout << "tx: " << tx << " ty: " << ty << " tz: " << tz << endl;
    }
    else
    {
      cout << "Velodyne scan failed to converge with the reference scan" << endl;
    }

  }


  if (monitor) {
    monitor->stop(); // This automatically makes both queues non-empty.

    cout << "Clearing LiDAR datagram queue." << endl;

    while (!laser_queue_end) {
      auto datagram = monitor->getLaserQueue().pop();
      if (datagram.laser_bank_measures[0].start_identifier == 0) {
        laser_queue_end = true;
      }
    }

    cout << "Clearing LiDAR position datagram queue." << endl;

    while (!position_queue_end) {
      auto datagram = monitor->getPositionQueue().pop();
      if (isFinalPositionPacket(datagram)) {
        position_queue_end = true;
      }
    }

    delete monitor;
  }

}
