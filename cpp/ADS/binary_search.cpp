#include <span>
#include <vector>
#include <cstdint>
#include <cassert>

auto upper_bound(std::span<int32_t> v, int32_t x) {
  auto L = -1;
  auto R = v.size();
  while (R - L > 1) {
    auto m = L + (R - L) / 2;
    if (v[m] >= x) {
      R = m;
    } else {
      L = m;
    }
  }
  return R;
}

auto lower_bound(std::span<int32_t> v, int32_t x) {
  auto L = -1;
  auto R = v.size();
  while (R - L > 1) {
    auto m = L + (R - L) / 2;
    if (v[m] > x) {
      R = m;
    } else {
      L = m;
    }
  }
  return L;
}

int main() {
  std::vector<int32_t> v{-12, -11, -9, -8, -6, -3, 0, 2, 4, 5, 6, 8, 10};

  assert(upper_bound(v, -12) == 0);
  assert(upper_bound(v, -11) == 1);
  assert(upper_bound(v, -10) == 2);
  assert(upper_bound(v, -9) == 2);

  assert(lower_bound(v, -12) == 0);
  assert(lower_bound(v, -11) == 1);
  assert(lower_bound(v, -10) == 1);
  assert(lower_bound(v, -9) == 2);

  return 0;
}