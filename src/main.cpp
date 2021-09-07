/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"

#include <signal.h>

//###################################################
//                              COUT STANDARD MESSAGE
//###################################################
/**
   \fn message(const T& msg, T1 val = T1())
   \brief Convenience method to display text
*/
template<typename T, typename T1>
void message(const T &msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

//###################################################
//                                               MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
const std::string name_node = "a_star";

void SigHandler(int sig) {
  std::cerr << name_node + " died with #" + std::to_string(sig) << " - " << strsignal(sig) << std::endl;
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {

  message<string, int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");

  message("cell size: ", HybridAStar::Constants::cellSize);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  ros::init(argc, argv, name_node);

  std::vector<int> signals{
      SIGINT,
      SIGILL,
      SIGABRT,
      SIGFPE,
      SIGSEGV,
      SIGTERM,
      SIGHUP,
      SIGQUIT,
      SIGTRAP,
      SIGKILL,
      SIGBUS,
      SIGSYS,
      SIGPIPE,
      SIGALRM,
      SIGURG,
      SIGSTOP,
      SIGTSTP,
      SIGCONT,
      SIGCHLD,
      SIGTTIN,
      SIGTTOU,
      SIGPOLL,
      SIGXCPU,
      SIGXFSZ,
      SIGVTALRM,
      SIGPROF
  };

  for (const auto &sig : signals) {
    signal(sig, SigHandler);
  }

  HybridAStar::Planner hy;
  hy.plan();

  ros::spin();
  return 0;
}
