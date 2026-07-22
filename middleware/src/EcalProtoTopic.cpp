#include <middleware/EcalProtoTopic.hpp>

#include <ecal/ecal.h>

#include <mutex>
#include <string>

namespace middleware {
namespace {

std::mutex g_ecal_mutex;
int g_ecal_ref_count = 0;
std::string g_ecal_process_name;

} // namespace

void retain_ecal(const std::string& process_name) {
  std::scoped_lock lock(g_ecal_mutex);
  if (g_ecal_ref_count == 0) {
    g_ecal_process_name = process_name.empty() ? std::string{"camera_tracking"} : process_name;
    
    eCAL::Initialize(0, nullptr, g_ecal_process_name.c_str());
  }
  ++g_ecal_ref_count;
}

void release_ecal() {
  std::scoped_lock lock(g_ecal_mutex);
  if (g_ecal_ref_count > 0) {
    --g_ecal_ref_count;
    if (g_ecal_ref_count == 0) {
      eCAL::Finalize();
      g_ecal_process_name.clear();
    }
  }
}

} // namespace middleware
