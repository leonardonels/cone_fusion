#pragma once

#include <algorithm>
#include <map>
#include <stdint.h>

#define blueCone 0
#define yellowCone 1
#define orangeCone 2
#define orangeBigCone 3

enum class ColorId : uint8_t {
  blue_cone,
  yellow_cone,
  orange_cone,
  orange_big_cone,
  end,
};

class ColorLogic {
public:
  explicit ColorLogic() {
    /* Initialize color counter */
    for (uint8_t i = 0; i != static_cast<uint8_t>(ColorId::end); ++i)
      cone_info[static_cast<ColorId>(i)] = 0;
  }

  /* Increments the color counter */
  void setColor(uint8_t color_id) {

    ColorId color = static_cast<ColorId>(color_id);

    if (cone_info.find(color) == cone_info.end()) {
      return;
    }

    ++cone_info.at(color);
    ++times_detected_;
  }

  /* Return the color of the most seen cone */
  ColorId getConeColor() {
    auto output_cone =
        std::max_element(cone_info.begin(), cone_info.end(),
                         [](const std::pair<ColorId, uint32_t> &p1,
                            const std::pair<ColorId, uint32_t> &p2) {
                           return p1.second < p2.second;
                         });

    return output_cone->first;
  }

  /* Return the color of the most seen cone + the times it has been seen */
  std::pair<ColorId, uint32_t> getConeColorAndCount() {
    auto output_cone =
        std::max_element(cone_info.begin(), cone_info.end(),
                         [](const std::pair<ColorId, uint32_t> &p1,
                            const std::pair<ColorId, uint32_t> &p2) {
                           return p1.second < p2.second;
                         });
    return *output_cone;
  }

  /* Detection / visibility counters for false-positive rejection. A landmark is
     "detected" each time it is associated to an observation (setColor) and
     "expected" each scan it lies within the sensor FOV/range. The ratio
     detected/expected is ~1 for a real cone (seen on nearly every pass it is
     visible) and stays low for a ghost that only matches sporadically across
     laps, which a plain cumulative count cannot distinguish. */
  void incrementExpected() { ++times_expected_; }
  uint32_t getDetected() const { return times_detected_; }
  uint32_t getExpected() const { return times_expected_; }

private:
  std::map<ColorId, uint32_t> cone_info;
  uint32_t times_detected_ = 0;  /* total associations (detections) of this landmark */
  uint32_t times_expected_ = 0;  /* scans this landmark was within sensor FOV/range */
};