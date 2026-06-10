# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/PIC18.cmf"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/PIC18.hex"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/PIC18.hxl"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/PIC18.mum"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/PIC18.o"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/PIC18.sdb"
  "/Users/kevinlu/Code/Github/MPLABXProjects/leaksense_counter.X/out/leaksense_counter/PIC18.sym"
  )
endif()
