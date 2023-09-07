#include "gtest/gtest.h"
#include "samplemodule.hpp"

TEST(SampleModulePkg, testGetObjectIds){
  
  auto sm = SampleModule();
  std::vector<std::uint32_t> objectIds = sm.getObjectIds();
  EXPECT_EQ(objectIds.size(), 0);
}
