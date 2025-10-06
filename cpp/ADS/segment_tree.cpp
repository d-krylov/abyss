#include <vector>
#include <cstdint>
#include <span>
#include <cassert>
#include <print>

class SegmentTree {
public:
  SegmentTree(std::span<const int32_t> v) : size_(v.size()) {
    auto pow2 = 1u << std::bit_width(size_ - 1);
    segment_tree_.resize(pow2, 0);
    Build(v, 0, 0, size_);
  }

  int32_t Sum(int32_t a, int32_t b) const {
    return Sum(0, 0, size_, a, b);
  }

protected:
  void Build(std::span<const int32_t> v, uint32_t index, uint32_t L, uint32_t R) {
    if (R - L == 1)
      segment_tree_[index] = v[L];
    else {
      auto M = L + (R - L) / 2;
      Build(v, 2 * index + 1, L, M);
      Build(v, 2 * index + 2, M, R);
      segment_tree_[index] = segment_tree_[2 * index + 1] + segment_tree_[2 * index + 2];
    }
  }

  int32_t Sum(uint32_t index, int32_t L, int32_t R, int32_t a, int32_t b) const {
    if (b <= L || R <= a) return 0;
    if (a <= L && R <= b) return segment_tree_[index];
    auto M = L + (R - L) / 2;
    return Sum(2 * index + 1, L, M, a, b) + Sum(2 * index + 2, M, R, a, b);
  }

private:
  std::size_t size_{0};
  std::vector<int32_t> segment_tree_;
};

int main() {

  std::vector<int32_t> v{-1, -2, -3, -4, -5, 1, 2, 3, 4, 5};

  SegmentTree segment_tree(v);

  std::println("{}", segment_tree.Sum(0, 10));

  assert(segment_tree.Sum(0, 3) == -6);
  assert(segment_tree.Sum(1, 4) == -9);
  assert(segment_tree.Sum(2, 5) == -12);
  assert(segment_tree.Sum(5, v.size()) == 15);
  assert(segment_tree.Sum(0, v.size()) == 0);

  return 0;
}