# The following variables contains the files used by the different stages of the build process.
set(PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_assemble)
set_source_files_properties(${PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_assemble})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_assemblePreprocess)
set_source_files_properties(${PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_assemblePreprocess} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_assemblePreprocess})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_compile
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Compress_NoComp_2B_2B.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Compress_Pack_10_10_4.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Dev_Led.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Dev_Uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FlowLog.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FlowMeter.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../FlowReport.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../MCU_Time.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../PulseCounter.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../Sys_Time_MCU_Specific.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../led_fsm_sysstate.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../main.c")
set_source_files_properties(${PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(PIC18F06Q40_Template_default_default_XC8_FILE_TYPE_link)
set(PIC18F06Q40_Template_default_image_name "default.elf")
set(PIC18F06Q40_Template_default_image_base_name "default")

# The output directory of the final image.
set(PIC18F06Q40_Template_default_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/PIC18F06Q40_Template")

# The full path to the final image.
set(PIC18F06Q40_Template_default_full_path_to_image ${PIC18F06Q40_Template_default_output_dir}/${PIC18F06Q40_Template_default_image_name})

# Potential output file extensions
set(output_extensions
    .hex
    .hxl
    .mum
    .o
    .sdb
    .sym
    .cmf)
list(TRANSFORM output_extensions PREPEND "${PIC18F06Q40_Template_default_output_dir}/${PIC18F06Q40_Template_default_image_base_name}")
