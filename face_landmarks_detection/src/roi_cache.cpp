// Copyright (c) 2024, TCL.
// Licensed under the Apache License, Version 2.0.

#include "roi_cache.h"
#include <algorithm>

void RoiCache::Update(int x1, int y1, int x2, int y2, int track_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  CachedRoi roi;
  roi.x1 = x1;
  roi.y1 = y1;
  roi.x2 = x2;
  roi.y2 = y2;
  roi.track_id = track_id;
  roi.timestamp = std::chrono::steady_clock::now();
  cached_roi_ = roi;
}

bool RoiCache::IsValid() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cached_roi_.has_value()) {
    return false;
  }
  auto now = std::chrono::steady_clock::now();
  auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - cached_roi_->timestamp).count();
  return age_ms <= max_age_ms_;
}

#ifdef BPU_LIBDNN
std::optional<hbDNNRoi> RoiCache::GetExpandedRoi(int img_width, int img_height) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cached_roi_.has_value()) {
    return std::nullopt;
  }

  auto now = std::chrono::steady_clock::now();
  auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - cached_roi_->timestamp).count();
  if (age_ms > max_age_ms_) {
    return std::nullopt;
  }

  const auto& roi = cached_roi_.value();
  int width = roi.x2 - roi.x1;
  int height = roi.y2 - roi.y1;
  int cx = (roi.x1 + roi.x2) / 2;
  int cy = (roi.y1 + roi.y2) / 2;

  int new_width = static_cast<int>(width * expand_scale_);
  int new_height = static_cast<int>(height * expand_scale_);

  hbDNNRoi expanded;
  expanded.left = std::max(0, cx - new_width / 2);
  expanded.top = std::max(0, cy - new_height / 2);
  expanded.right = std::min(img_width - 1, cx + new_width / 2);
  expanded.bottom = std::min(img_height - 1, cy + new_height / 2);

  return expanded;
}
#endif
