#ifndef MIMZY_STACK_H
#define MIMZY_STACK_H

#include <cstdint>

namespace Mimzy {

template <typename DataType, int N> class Stack {
public:
  void Push(const DataType &data) {
    stack_[position_++] = data;
  }

  DataType Pop() {
    return stack_[--position_];
  }

  auto Empty() {
    return position_ == 0;
  }

private:
  DataType stack_[N] = {};
  uint32_t position_ = {};
};

} // namespace Mimzy

#endif // MIMZY_STACK_H