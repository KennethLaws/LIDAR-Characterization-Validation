/*************************************************************
  Generalized-ICP Copyright (c) 2009 Aleksandr Segal.
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

 * Redistributions of source code must retain the above 
 copyright notice, this list of conditions and the 
 following disclaimer.
 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the 
 following disclaimer in the documentation and/or other
 materials provided with the distribution.
 * The names of the contributors may not be used to endorse
 or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 DAMAGE.
 *************************************************************/



#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

// program options
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/tokenizer.hpp> 

#include "gicp.h"

using namespace std;
using namespace dgc::gicp;
namespace po = boost::program_options;

static stringstream ss_filename1;
static stringstream ss_filename2;

static string filename1;
static string filename2;
static string filename_t_base;
static bool debug = false;
static double gicp_epsilon = 1e-3;
static double max_distance = 5.;


void print_usage(const char* program_name, po::options_description const& desc) {
    cout << program_name << " [options] scan1 scan2 [t_base]" << endl;
    cout << desc << endl;  
}


bool parse_options(int argc, char** argv) {
    bool error = false;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "print help message")
        ("scan1", po::value<string>(), "scan1 filename")
        ("scan2", po::value<string>(), "scan2 filename")
        ("t_base", po::value<string>(), "base transform filename")
        ("debug", po::value<bool>(), "enable debug mode")
        ("epsilon", po::value<double>(), "G-ICP epsilon constant")
        ("d_max", po::value<double>(), "maximum distance for matching points");

    po::positional_options_description p;
    p.add("scan1", 1);
    p.add("scan2", 1);
    p.add("t_base", 1);

    po::variables_map vm;
    try {    
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        po::notify(vm);
    }
    catch(const std::exception &e) {
        cout << "Error: " << e.what() << endl;
        print_usage(argv[0], desc);
        error = true;
        return error;
    }

    if(vm.count("help")) {
        print_usage(argv[0], desc);
        error = true;
        return error;
    }

    if(!vm.count("scan1")) {
        cout << "Error: scan1 filename not specified!" << endl;
        error = true;
    }
    else {
        filename1 = vm["scan1"].as<string>();
    }

    if(!vm.count("scan2")) {
        cout << "Error: scan2 filename not specified!" << endl;
        error = true;
    }
    else {
        filename2 = vm["scan2"].as<string>();
    }

    if(error) {
        print_usage(argv[0], desc);
    }


    if(vm.count("debug")) {
        debug = vm["debug"].as<bool>();
    }

    if(vm.count("epsilon")) {
        gicp_epsilon = vm["epsilon"].as<double>();
    }

    if(vm.count("d_max")) {
        max_distance = vm["d_max"].as<double>();
    } 

    if(vm.count("t_base")) {
        filename_t_base = vm["t_base"].as<string>();
    }

    return error;
}

bool load_points(GICPPointSet *set, const char* filename) {
    bool error = false;

    ifstream in(filename);
    if(!in) {
        cout << "Could not open '" << filename << "'." << endl;
        return true;
    }
    string line;
    GICPPoint pt;
    pt.range = -1;
    for(int k = 0; k < 3; k++) {
        for(int l = 0; l < 3; l++) {
            pt.C[k][l] = (k == l)?1:0;
        }
    }    
    while(getline(in, line)) {
        istringstream sin(line);
        sin >> pt.x >> pt.y >> pt.z;    
        if (pt.x != 0 && pt.y != 0 && pt.z != 0)
        {
            set->AppendPoint(pt);    
        }
    }
    in.close();

    return false;  
};

int main(int argc, char** argv) 
{
    FILE* fid = fopen("sm_results_HT033_1401525581.txt", "w");
    FILE* fid_rpy = fopen("sm_rt_results_HT033_1401525581.txt", "w");
    bool error = false;  
    double rx,ry,rz,tx,ty,tz;    
    /*bool error = parse_options(argc, argv);

      if(error) {
      return 1;
      }*/

    GICPPointSet p1, p2;
    dgc_transform_t t_base, t0, t1, t_result;

    dgc_transform_identity(t_result);

    for (int i = 1; i < 5129; i++)
    {
        // set up the transformations
        dgc_transform_identity(t_base);
        dgc_transform_identity(t0);
        dgc_transform_identity(t1);

        if ( i > 1)
        {
            dgc_transform_copy(t_base, t_result);
        }

        cout << "Setting up pointclouds..." << endl;

        ss_filename1 << "data/HT033_1401525581/" << i << "_pose.ascii";
        ss_filename2 << "data/HT033_1401525581/" << i+1 << "_pose.ascii";

        filename1 = ss_filename1.str();
        filename2 = ss_filename2.str();

        error = load_points(&p1, filename1.c_str());
        if(error) {
            return 1;
        }
        cout << "Loaded " << filename1 << ": " << p1.Size() << " points into GICPPointSet 1." << endl;
        error = load_points(&p2, filename2.c_str());
        if(error) {
            return 1;
        }
        cout << "Loaded " << filename2 << ": " << p2.Size() << " points into GICPPointSet 2." << endl;  

        // build kdtrees and normal matrices
        cout << "Building KDTree and computing surface normals/matrices..." << endl;

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
        cout << "Scans : " << i << " & " << i+1 << " Converged in : " << iterations << " iterations" << endl;
        dgc_transform_print(t_base, "t_base");
        dgc_transform_print(t0, "t0");  
        dgc_transform_print(t1, "t1");

        dgc_transform_left_multiply(t_result, t1);

        dgc_transform_print(t_result, "t_result");

        dgc_transform_get_translation(t_result, &tx, &ty, &tz);
        dgc_transform_get_rotation(t_result, &rx, &ry, &rz);

        fprintf(fid_rpy,"%f %f %f %f %f %f %d\n", rx, ry, rz, tx, ty, tz, iterations);

        fprintf(fid,"%f %f %f %f\t%f %f %f %f\t%f %f %f %f\t%f %f %f %f\n",t_result[0][0],t_result[0][1],t_result[0][2],t_result[0][3],
                t_result[1][0],t_result[1][1],t_result[1][2],t_result[1][3],t_result[2][0],t_result[2][1],t_result[2][2],
                t_result[2][3],t_result[3][0],t_result[3][1],t_result[3][2],t_result[3][3]);

        ss_filename1.str(std::string());
        ss_filename2.str(std::string());

        if (iterations > 99)
        {
            dgc_transform_identity(t_result);
        }

        p1.Clear();
        p2.Clear();

    }

    fclose(fid);

    return 0;
}
