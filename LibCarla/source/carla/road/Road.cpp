// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/Road.h"
#include "carla/road/Lane.h"
#include "carla/road/MapData.h"
#include "carla/Logging.h"
#include "carla/ListView.h"

#include <boost/optional.hpp>

namespace carla {
namespace road {

  const MapData *Road::GetMap() const {
    return _map_data;
  }

  RoadId Road::GetId() const {
    return _id;
  }

  std::string Road::GetName() const {
    return _name;
  }

  float Road::GetLength() const {
    return _length;
  }

  bool Road::IsJunction() const {
    return _is_junction;
  }

  JuncId Road::GetJunction() const {
    return _junction_id;
  }

  std::vector<const Road *> Road::GetNexts() const {
    std::vector<const Road *> vec;
    for (auto &&next : _nexts) {
      vec.emplace_back(_map_data->GetRoad(next));
    }
    return vec;
  }

  std::vector<const Road *> Road::GetPrevs() const {
    std::vector<const Road *> vec;
    for (auto &&prev : _prevs) {
      vec.emplace_back(_map_data->GetRoad(prev));
    }
    return vec;
  }

  Lane *Road::GetLane(const LaneId id, const float s) {
    Road *me = this;
    for (auto &lane_section : me->GetLaneSectionsAt(s)) {
      auto search = lane_section.GetLanes().find(id);
      if (search != lane_section.GetLanes().end()) {
        return &search->second;
      }
    }
    log_warning("id", id, "at distance", s, "not found in road", _id);
    return nullptr;
  }

  carla::road::signal::Signal* Road::GetSignal(const SignId id) {
    auto search = _signals.find(id);
    if (search != _signals.end()) {
      return &search->second;
    }
    return nullptr;
  }

  carla::road::signal::SignalReference* Road::GetSignalRef(const SignRefId id) {
    const auto search = _sign_ref.find(id);
    if (search != _sign_ref.end()) {
      return &search->second;
    }
    return nullptr;
  }

  std::unordered_map<SignId, signal::Signal>* Road::getSignals() {
    return &_signals;
  }

  std::unordered_map<SignId, signal::SignalReference>* Road::getSignalReferences() {
    return &_sign_ref;
  }

} // road
} // carla