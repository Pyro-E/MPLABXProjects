# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/Users/kevinlu/Code/Github/MPLABXProjects/Blinky.X/out/Blinky/default.cmf"
  "/Users/kevinlu/Code/Github/MPLABXProjects/Blinky.X/out/Blinky/default.hex"
  "/Users/kevinlu/Code/Github/MPLABXProjects/Blinky.X/out/Blinky/default.hxl"
  "/Users/kevinlu/Code/Github/MPLABXProjects/Blinky.X/out/Blinky/default.mum"
  "/Users/kevinlu/Code/Github/MPLABXProjects/Blinky.X/out/Blinky/default.o"
  "/Users/kevinlu/Code/Github/MPLABXProjects/Blinky.X/out/Blinky/default.sdb"
  "/Users/kevinlu/Code/Github/MPLABXProjects/Blinky.X/out/Blinky/default.sym"
  )
endif()
