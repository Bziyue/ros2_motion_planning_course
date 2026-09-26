#include <iostream>
#include "sample_stamp.hpp"

int main()
{
  for (const std::int64_t k : {0LL, 1LL, 2LL, 7LL, 25200LL, 495000LL}) {
    if (studentSampleStamp(k, 7) != (k * 1000000000LL + 3) / 7 ||
      studentSampleStamp(k, 137.5) != (k * 2000000000LL + 137) / 275 ||
      studentSampleStamp(k, 200) != k * 5000000)
    {
      std::cerr << "FAIL: use sample index, seconds-to-nanoseconds and nearest rounding\n";
      return 1;
    }
  }
  std::cout << "PASS: aligned/nonaligned rates, t=0 and long-run phase\n";
}
