# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/default.cmf"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/default.hex"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/default.hxl"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/default.mum"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/default.o"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/default.sdb"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/default.sym"
  )
endif()
