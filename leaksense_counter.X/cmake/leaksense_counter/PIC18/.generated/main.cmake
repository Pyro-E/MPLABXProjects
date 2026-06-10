include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(leaksense_counter_PIC18_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group PIC18_toolchain
if(leaksense_counter_PIC18_PIC18_toolchain_FILE_TYPE_assemble)
add_library(leaksense_counter_PIC18_PIC18_toolchain_assemble OBJECT ${leaksense_counter_PIC18_PIC18_toolchain_FILE_TYPE_assemble})
    leaksense_counter_PIC18_PIC18_toolchain_assemble_rule(leaksense_counter_PIC18_PIC18_toolchain_assemble)
    list(APPEND leaksense_counter_PIC18_library_list "$<TARGET_OBJECTS:leaksense_counter_PIC18_PIC18_toolchain_assemble>")

endif()

# Handle files with suffix S, for group PIC18_toolchain
if(leaksense_counter_PIC18_PIC18_toolchain_FILE_TYPE_assemblePreprocess)
add_library(leaksense_counter_PIC18_PIC18_toolchain_assemblePreprocess OBJECT ${leaksense_counter_PIC18_PIC18_toolchain_FILE_TYPE_assemblePreprocess})
    leaksense_counter_PIC18_PIC18_toolchain_assemblePreprocess_rule(leaksense_counter_PIC18_PIC18_toolchain_assemblePreprocess)
    list(APPEND leaksense_counter_PIC18_library_list "$<TARGET_OBJECTS:leaksense_counter_PIC18_PIC18_toolchain_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group PIC18_toolchain
if(leaksense_counter_PIC18_PIC18_toolchain_FILE_TYPE_compile)
add_library(leaksense_counter_PIC18_PIC18_toolchain_compile OBJECT ${leaksense_counter_PIC18_PIC18_toolchain_FILE_TYPE_compile})
    leaksense_counter_PIC18_PIC18_toolchain_compile_rule(leaksense_counter_PIC18_PIC18_toolchain_compile)
    list(APPEND leaksense_counter_PIC18_library_list "$<TARGET_OBJECTS:leaksense_counter_PIC18_PIC18_toolchain_compile>")

endif()


# Main target for this project
add_executable(leaksense_counter_PIC18_image_bFs_k2cg ${leaksense_counter_PIC18_library_list})

set_target_properties(leaksense_counter_PIC18_image_bFs_k2cg PROPERTIES
    OUTPUT_NAME "PIC18"
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES "${output_extensions}"
    RUNTIME_OUTPUT_DIRECTORY "${leaksense_counter_PIC18_output_dir}")
target_link_libraries(leaksense_counter_PIC18_image_bFs_k2cg PRIVATE ${leaksense_counter_PIC18_PIC18_toolchain_FILE_TYPE_link})

# Add the link options from the rule file.
leaksense_counter_PIC18_link_rule( leaksense_counter_PIC18_image_bFs_k2cg)


