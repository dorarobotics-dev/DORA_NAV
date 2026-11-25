extern "C" {
#include "node_api.h"
}

#include "karto_dora/message_adapter.hpp"
#include "karto_dora/node_config.hpp"
#include "karto_dora/runtime.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#ifndef KARTO_DEFAULT_CONFIG
#define KARTO_DEFAULT_CONFIG "../modules/mapping/karto_slam_dora/config/config.yaml"
#endif

namespace {
double Now()
{
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Flush(void *context, karto_dora::MappingRuntime &runtime)
{
  for (const auto &output : runtime.TakeOutputs()) {
    std::string id = output.id;
    std::string payload = output.payload;
    if (id == "CorrectedPose") {
      id = "pose";
      payload = karto_dora::ToDoraPoseMessage(payload);
    } else if (id == "SlamStatus") {
      id = "slam_status";
    } else if (id == "SafetyStop") {
      id = "safety_stop";
    }
    if (dora_send_output(context, const_cast<char *>(id.data()), id.size(),
                         const_cast<char *>(payload.data()), payload.size()) != 0) {
      throw std::runtime_error("dora_send_output failed for " + id);
    }
  }
}
}  // namespace

int main()
{
  void *context = nullptr;
  try {
    context = init_dora_context_from_env();
    if (!context) throw std::runtime_error("failed to initialize Dora context");
    const auto settings = karto_dora::LoadNodeSettings(KARTO_DEFAULT_CONFIG);
    karto_dora::RuntimeOptions options;
    options.map_prefix = settings.map_prefix;
    options.pose_log_path = settings.pose_log_path;
    karto_dora::MappingRuntime runtime(settings.slam, options);
    Flush(context, runtime);
    while (void *event = dora_next_event(context)) {
      const auto type = read_dora_event_type(event);
      if (type == DoraEventType_Stop) {
        runtime.Stop();
        Flush(context, runtime);
        free_dora_event(event);
        break;
      }
      if (type == DoraEventType_Input) {
        char *id = nullptr;
        size_t id_size = 0;
        read_dora_input_id(event, &id, &id_size);
        if (std::string_view(id, id_size) == "tick") {
          runtime.Tick(Now());
        } else {
          char *data = nullptr;
          size_t data_size = 0;
          read_dora_input_data(event, &data, &data_size);
          runtime.HandleInput(std::string_view(id, id_size),
                              std::string_view(data, data_size), Now());
        }
        Flush(context, runtime);
      }
      free_dora_event(event);
    }
    // Stop is idempotent, including after an explicit Dora Stop event.
    const bool stopped_cleanly = runtime.Stop();
    Flush(context, runtime);
    if (!stopped_cleanly) {
      std::cerr << "karto_slam_dora_node: map save failed during shutdown\n";
    }
    free_dora_context(context);
    return stopped_cleanly ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << "karto_slam_dora_node error: " << error.what() << '\n';
    if (context) free_dora_context(context);
    return 1;
  }
}
