#pragma once

#include <string>
#include <vector>

#include "Core/MPMbox.hpp"
#include "Core/ProcessedDataMP.hpp"

struct PostProcessor;
class PBC3Dbox;

//
// Drives a post-processing run.
//
// A PostSession reads a command file, which declares
//   - where the configuration files are and where the results go,
//   - the range of configurations to walk through,
//   - the list of actions ('Post' lines) to apply to each of them,
// then loads the configurations one after the other and lets every action
// extract what it needs.
//
// The class holds the loaded configuration, so an action reaches the data
// through its 'session' pointer rather than through globals -- which is the
// difference with the older 'See/cut.cpp'.
//
class PostSession {
public:
  MPMbox Conf; // The configuration currently loaded

  // The per-point fields of that configuration, either projected on the grid
  // and brought back (the default, what 'see' displays) or taken raw from the
  // Material Points. The 'smoothing' command of the command file decides.
  std::vector<ProcessedDataMP> Data;

  std::string sourceFolder{"."}; // Where the conf<N>.txt files are read
  std::string resultFolder{"."}; // Where the output files are written

  // Range of configurations, as given by the 'confs' command.
  int confFirst{0};
  int confLast{0};
  int confStride{1};

  int confNum{0};       // Number of the configuration currently loaded
  double confTime{0.0}; // Its time, kept aside because Conf is cleaned between reads
  bool verbose{true};

  // false: use the values the Material Points carry, without the projection
  // on the grid. See useRawMPData() for what that does and does not change.
  bool smoothing{true};

  std::vector<PostProcessor *> processors;

  PostSession();
  ~PostSession();

  // Registers the available actions to the factory.
  void ExplicitRegistrations();

  // Reads the command file. Returns false if it cannot be opened.
  bool read(const char *commandFile);

  // Loads sourceFolder/conf<num>.txt and fills Data.
  // Returns false when the file does not exist.
  bool loadConf(int num);

  // Puts back into Data the values the Material Points carry, undoing the
  // grid projection for the three fields that projection actually changes.
  void useRawMPData();

  // Loads sourceFolder/DEM_MP<p>/conf<confNum> into 'dem'. Returns false when
  // the file does not exist -- which is the normal case for a Material Point
  // that was not tracked, or for a computation with no double scale at all.
  bool loadDEMConf(size_t p, PBC3Dbox &dem);

  // Walks through the configurations and calls the actions.
  void run();

  std::string confPath(int num) const;
  std::string demConfPath(size_t p, int num) const;
  std::string outPath(const std::string &name) const;
};
