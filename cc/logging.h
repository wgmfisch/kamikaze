#include <iostream>

struct NoAssertHelper {};

struct AssertHelper {
  ~AssertHelper() { abort(); }

  operator NoAssertHelper() const { return NoAssertHelper(); }
};

template <typename T>
const AssertHelper &operator<<(const AssertHelper &helper, T &&msg) {
  std::cerr << std::forward<T>(msg);
  return helper;
}

#define QCHECK(cond)                                                           \
  ((cond)) ? NoAssertHelper() : AssertHelper() << __FILE__ << ":" << __LINE__  \
                                               << ": " << #cond

