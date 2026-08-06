#include <ctime>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <regex>

#include "toofus-gate/tclap/CmdLine.h"

#include "Core/MPMbox.hpp"
#include "ExecChrono.hpp"
#include "Spies/Spy.hpp"
#include "stackTracer.hpp"

ExecChrono SimuChrono;
MPMbox *SimuHandler;

// Names of the signals StackTracer installs a handler for.
static const char *signalName(int sig) {
  switch (sig) {
    case SIGABRT: return "(Abort) Abnormal termination, such as is initiated by the abort function.";
    case SIGFPE: return "(Floating-Point Exception) Erroneous arithmetic operation.";
    case SIGILL: return "(Illegal Instruction) Invalid function image.";
    case SIGINT: return "(Interrupt) Interactive attention signal.";
    case SIGSEGV: return "(Segmentation Violation) Invalid access to storage.";
    case SIGTERM: return "(Terminate) Termination request sent to program.";
    default: return "(Unknown signal)";
  }
}

void mySigHandler(int sig) {
  std::cerr << std::endl << std::endl;
  SimuChrono.stop();

  for (size_t s = 0; s < SimuHandler->Spies.size(); ++s) { SimuHandler->Spies[s]->end(); }

  /*
  std::cerr << "Timer data saved in 'stopped' file\n";
outputManager tmp_out("stopped");
tmp_out.writeFile();
tmp_out.printTimeTable();
*/

  // SimuHandler->save_state("data", SimuHandler->step);

  // StackTracer::defaultSigHandler would print the trace and then call
  // exit(EXIT_SUCCESS): the process would end with the code 0, and nothing that
  // drives mpmbox -- a script, make, a batch scheduler -- could tell a
  // computation carried to its end from one that blew up at the third step. The
  // trace is printed here instead, followed by the conventional code for a death
  // by signal. (The exit(EXIT_SUCCESS) is in deps/toofus-src/stackTracer.hpp,
  // which is fetched by CMake, so it is not ours to patch.)
  std::cerr << std::endl << "The Simulation received the following signal:" << std::endl;
  std::cerr << signalName(sig) << std::endl;
  if (sig != SIGINT) { std::cerr << StackTracer::StackTrace() << std::endl; }

  // _Exit rather than exit: we are inside a signal handler, where destructors and
  // atexit handlers are not safe to run.
  std::_Exit(128 + sig);
}

//
//  Deletes files matching the pattern 'conf*.txt' and other files in the current directory.
//
void cleanSimulationFolder() {
  std::vector<std::regex> patterns = {std::regex(R"(perflog_[a-zA-Z0-9]+_\d+\.txt)"), std::regex(R"(conf\d+\.txt)")};

  size_t deletedCount = 0;

  for (const auto &entry : std::filesystem::directory_iterator(".")) {
    if (!std::filesystem::is_regular_file(entry)) continue;

    const std::string filename = entry.path().filename().string();

    for (const auto &pattern : patterns) {
      if (std::regex_match(filename, pattern)) {
        std::filesystem::remove(entry.path());
        // std::cout << "Deleted: " << filename << std::endl;
        deletedCount++;
        break; // avoid matching multiple patterns
      }
    }
  }

  std::cout << "Total files deleted: " << deletedCount << std::endl;
}

int main(int argc, char **argv) {
  INIT_TIMERS();
  time_t rawtime;
  time(&rawtime);
  printf("The current local time is: %s", ctime(&rawtime));

  std::string confFileName;
  int nbThreads            = 1;
  int verboseLevel         = 0;
  bool cleanAndLeave       = false;
  bool printBannerAndLeave = false;

  try {

    TCLAP::CmdLine cmd("This is the command line interface for MPMbox", ' ', "0.5");
    TCLAP::SwitchArg cleanArg("c", "clean", "Clean files", false);
    TCLAP::SwitchArg bannerArg("b", "banner", "show banner", false);
    TCLAP::UnlabeledValueArg<std::string> nameArg("input", "Name of the conf-file", false, "conf0", "conf-file");
    TCLAP::ValueArg<int> nbThreadsArg("j", "nbThreads", "Number of threads to be used", false, 1, "int");
    TCLAP::ValueArg<int> verboseArg("v", "verbose", "Verbose level", false, 4, "int");

    cmd.add(cleanArg);
    cmd.add(bannerArg);
    cmd.add(nameArg);
    cmd.add(nbThreadsArg);
    cmd.add(verboseArg);

    cmd.parse(argc, argv);

    cleanAndLeave       = cleanArg.getValue();
    printBannerAndLeave = bannerArg.getValue();
    confFileName        = nameArg.getValue();
    nbThreads           = nbThreadsArg.getValue();
    verboseLevel        = verboseArg.getValue();

  } catch (TCLAP::ArgException &e) {
    std::cerr << "Tclap error: " << e.error() << " for argument " << e.argId() << std::endl;
  }

  if (cleanAndLeave) {
    cleanSimulationFolder();
    return 0;
  }

  MPMbox Simulation;
  Simulation.setVerboseLevel(verboseLevel);
  Simulation.showAppBanner();
  if (printBannerAndLeave) { return 0; }

  StackTracer::initSignals(mySigHandler);
  SimuHandler = &Simulation;

#ifdef _OPENMP
  omp_set_num_threads(nbThreads);
  Logger::info("OpenMP acceleration (Number of threads = {})", nbThreads);
#else
  Logger::info("No multithreading (Number of threads = {})", nbThreads);
#endif

  Simulation.read(confFileName.c_str());

  Logger::info("COMPUTATION STARTS");

  SimuChrono.start();
  Simulation.init();
  Simulation.run();

  SimuChrono.stop();
  Logger::info("COMPUTATION NORMALLY STOPPED");

  PRINT_TIMERS("mpmbox");

  return 0;
}
