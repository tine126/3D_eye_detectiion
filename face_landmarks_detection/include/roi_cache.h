// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#ifndef FACE_LANDMARKS_DETECTION__ROI_CACHE_H_
#define FACE_LANDMARKS_DETECTION__ROI_CACHE_H_

#include <optional>
#include <chrono>
#include <mutex>

#ifdef BPU_LIBDNN
#include "hb_dnn.h"
#endif

struct CachedRoi {
  int x1;
  int y1;
  int x2;
  int y2;
  int track_id;
  std::chrono::steady_clock::time_point timestamp;
};

class RoiCache {
 public:
  RoiCache() = default;
  ~RoiCache() = default;

  void Update(int x1, int y1, int x2, int y2, int track_id);

#ifdef BPU_LIBDNN
  std::optional<hbDNNRoi> GetExpandedRoi(int img_width, int img_height);
#endif

  bool IsValid() const;
  void SetExpandScale(float scale) { expand_scale_ = scale; }
  void SetMaxAgeMs(int max_age_ms) { max_age_ms_ = max_age_ms; }

 private:
  std::optional<CachedRoi> cached_roi_;
  float expand_scale_ = 1.2f;
  int max_age_ms_ = 100;
  mutable std::mutex mutex_;
};

#endif  // FACE_LANDMARKS_DETECTION__ROI_CACHE_H_
