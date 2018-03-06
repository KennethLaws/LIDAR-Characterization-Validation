LFLAGS += `pkg-config --libs gsl`
CXXFLAGS += `pkg-config --cflags gsl`

#set(CMAKE_CXX_FLAGS "-std=c++0x -isystem /opt/ros/groovy/include -pthread -Wall -Wextra")
LFLAGS += -L. -lgicp -Lann_1.1.1/lib -lANN -lgsl -lgslcblas \
      -lboost_program_options -lboost_system -lstdc++ -lrt \
      -lboost_filesystem -lboost_thread -lpthread
CXXFLAGS += -std=c++0x -isystem -pthread -O3 -I./ann_1.1.1/include/ANN -I./gsl-1.16 -I./ 

LINK = g++
CXX = g++

SOURCES = optimize.cpp gicp.cpp bfgs_funcs.cpp scan.cpp transform.c scan2ascii.cpp test_lidar.cpp VelodyneMonitor.cpp

BINARIES = test_gicp scan2ascii test_lidar

TARGETS = libgicp.a scan2ascii test_lidar

.SUFFIXES:
.SUFFIXES: .o .c .cpp .a

# rules
all: $(TARGETS)

libgicp.a: gicp.o optimize.o bfgs_funcs.o transform.o scan.o
	ar rvs $@ $^

test_lidar: test_lidar.o gicp.o optimize.o bfgs_funcs.o transform.o VelodyneMonitor.o

scan2ascii: scan.o scan2ascii.o transform.o

clean:
	rm -f *.o $(TARGETS) *~ t_*.tfm *.dat mahalanobis.txt correspondence.txt iterations.txt

$(BINARIES):
	$(LINK) -o $@ $^ $(LFLAGS)

.cpp.o: 
	$(CXX) $(CXXFLAGS) -c $< -o $@
