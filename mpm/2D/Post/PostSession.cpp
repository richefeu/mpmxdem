#include "PostSession.hpp"

#include <fstream>
#include <sstream>

#include "PostProcessor.hpp"

#include "DEMScalars.hpp"
#include "MPFields.hpp"
#include "Runout.hpp"
#include "Scalars.hpp"

#include "PBC3D.hpp"

PostSession::PostSession() { ExplicitRegistrations(); }

PostSession::~PostSession() {
  for (size_t i = 0; i < processors.size(); i++) { delete processors[i]; }
  processors.clear();
}

//
// Registers the post-processing actions to the factory, the same way
// MPMbox::ExplicitRegistrations() registers the Spies or the Schedulers.
// Add a line here when a new action is written.
//
void PostSession::ExplicitRegistrations() {
  Factory<PostProcessor, std::string>::Instance()->RegisterFactoryFunction(
      "Runout", [](void) -> PostProcessor * { return new Runout(); });
  Factory<PostProcessor, std::string>::Instance()->RegisterFactoryFunction(
      "MPFields", [](void) -> PostProcessor * { return new MPFields(); });
  Factory<PostProcessor, std::string>::Instance()->RegisterFactoryFunction(
      "DEMScalars", [](void) -> PostProcessor * { return new DEMScalars(); });
  Factory<PostProcessor, std::string>::Instance()->RegisterFactoryFunction(
      "Scalars", [](void) -> PostProcessor * { return new Scalars(); });
}

std::string PostSession::confPath(int num) const {
  return sourceFolder + fileTool::separator() + "conf" + std::to_string(num) + ".txt";
}

//
// The DEM configuration of a tracked Material Point, as written by
// MPMbox::run(): '<result_folder>/DEM_MP<p>/conf<iconf>' -- note that these
// files carry no '.txt' extension.
//
std::string PostSession::demConfPath(size_t p, int num) const {
  return sourceFolder + fileTool::separator() + "DEM_MP" + std::to_string(p) + fileTool::separator() + "conf" +
         std::to_string(num);
}

std::string PostSession::outPath(const std::string &name) const {
  return resultFolder + fileTool::separator() + name;
}

bool PostSession::read(const char *commandFile) {
  std::ifstream file(commandFile);
  if (!file) {
    Logger::error("@PostSession::read, cannot open the command file '{}'", commandFile);
    return false;
  }

  std::string token;
  file >> token;
  while (file) {
    if (token.empty() || token[0] == '/' || token[0] == '#' || token[0] == '!') {
      getline(file, token); // ignore the rest of the line

    } else if (token == "source_folder") {
      file >> sourceFolder;

    } else if (token == "result_folder") {
      file >> resultFolder;
      if (resultFolder != "" && resultFolder != "." && resultFolder != "./") {
        fileTool::create_folder(resultFolder);
      }

    } else if (token == "confs") {
      file >> confFirst >> confLast >> confStride;
      if (confStride <= 0) {
        Logger::warn("@PostSession::read, a stride of {} makes no sense, set back to 1", confStride);
        confStride = 1;
      }

    } else if (token == "verbose") {
      file >> verbose;

    } else if (token == "smoothing") {
      file >> smoothing;

    } else if (token == "Post") {
      std::string postName;
      file >> postName;
      PostProcessor *P = Factory<PostProcessor, std::string>::Instance()->Create(postName);
      if (P != nullptr) {
        P->plug(this);
        P->read(file);
        processors.push_back(P);
        Logger::info("Post action '{}' added", postName);
      } else {
        Logger::warn("@PostSession::read, the post-processing action '{}' is unknown", postName);
        getline(file, token); // its parameters would be read as commands otherwise
      }

    } else {
      Logger::warn("@PostSession::read, unknown token '{}'", token);
      getline(file, token);
    }

    file >> token;
  }

  return true;
}

//
// Loads a configuration file the way 'see' does it.
//
// The precaution on computationMode is the one taken in See/see.cpp and
// See/cut.cpp: MPMbox::read runs the Spy commands held by the conf-file, and
// a Spy opens its output file in write mode -- which would truncate the
// results of the computation we are post-processing.
//
bool PostSession::loadConf(int num) {
  std::string name = confPath(num);
  if (!fileTool::fileExists(name.c_str())) { return false; }

  Conf.computationMode = false;
  Conf.clean();
  Conf.read(name.c_str());

  // postProcess is run in both cases: it is what builds the element and shape
  // function arrays, and it is the only source of the velocity gradient.
  Conf.postProcess(Data);
  if (!smoothing) { useRawMPData(); }

  confNum  = num;
  confTime = Conf.t;
  return true;
}

//
// Undo the grid projection.
//
// Of everything MPMbox::postProcess puts in a ProcessedDataMP, only three
// fields are actually smoothed -- the velocity, the stress and its
// out-of-plane component. They are the ones sent to the nodes with the
// weights Np m_p / m_node and brought back with Np. The position, the
// deformation gradient (stored in the field named 'strain'), the density and
// the four corners are copied straight from the Material Point, so they are
// already raw and nothing has to be done to them.
//
// The velocity gradient is deliberately left alone. It has no raw
// counterpart: a Material Point does not carry one -- MPMbox::save does not
// even write MaterialPoint::velGrad to the conf-file -- and in the MPM a
// velocity gradient is a grid quantity by construction, obtained as
// sum_r gradN_r v_r. The shear rate is therefore the same in both modes. The
// inertial number is not, even so: I = gdot d / sqrt(P/rho_s) depends on the
// pressure, which does change.
//
void PostSession::useRawMPData() {
  for (size_t p = 0; p < Conf.MP.size(); p++) {
    Data[p].vel              = Conf.MP[p].vel;
    Data[p].stress           = Conf.MP[p].stress;
    Data[p].outOfPlaneStress = Conf.MP[p].outOfPlaneStress;
  }
}

bool PostSession::loadDEMConf(size_t p, PBC3Dbox &dem) {
  std::string name = demConfPath(p, confNum);
  if (!fileTool::fileExists(name.c_str())) { return false; }

  dem.clearMemory();
  dem.loadConf(name.c_str());
  dem.computeSampleData(); // fills Rmean, Vsolid, VelMean, etc.
  return true;
}

void PostSession::run() {
  if (processors.empty()) {
    Logger::warn("@PostSession::run, no 'Post' action was declared: nothing to do");
    return;
  }

  for (size_t i = 0; i < processors.size(); i++) { processors[i]->begin(); }

  int nbDone = 0;
  for (int num = confFirst; num <= confLast; num += confStride) {
    if (!loadConf(num)) {
      Logger::warn("conf{}.txt not found in '{}', skipped", num, sourceFolder);
      continue;
    }
    if (verbose) { Logger::info("conf{}  (t = {} s, {} MP)", num, Conf.t, Conf.MP.size()); }

    for (size_t i = 0; i < processors.size(); i++) { processors[i]->exec(); }
    nbDone++;
  }

  for (size_t i = 0; i < processors.size(); i++) { processors[i]->end(); }

  Logger::info("{} configuration(s) processed", nbDone);
}
