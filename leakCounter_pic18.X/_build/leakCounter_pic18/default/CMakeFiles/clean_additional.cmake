# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/Users/kevinlu/Code/Github/MPLABXProjects/leakCounter_pic18.X/out/leakCounter_pic18/default.cmf"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leakCounter_pic18.X/out/leakCounter_pic18/default.hex"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leakCounter_pic18.X/out/leakCounter_pic18/default.hxl"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leakCounter_pic18.X/out/leakCounter_pic18/default.mum"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leakCounter_pic18.X/out/leakCounter_pic18/default.o"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leakCounter_pic18.X/out/leakCounter_pic18/default.sdb"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leakCounter_pic18.X/out/leakCounter_pic18/default.sym"
  )
endif()
