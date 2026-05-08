include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(leaksense_counter_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(leaksense_counter_default_default_XC8_FILE_TYPE_assemble)
add_library(leaksense_counter_default_default_XC8_assemble OBJECT ${leaksense_counter_default_default_XC8_FILE_TYPE_assemble})
    leaksense_counter_default_default_XC8_assemble_rule(leaksense_counter_default_default_XC8_assemble)
    list(APPEND leaksense_counter_default_library_list "$<TARGET_OBJECTS:leaksense_counter_default_default_XC8_assemble>")

endif()

# Handle files with suffix S, for group default-XC8
if(leaksense_counter_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(leaksense_counter_default_default_XC8_assemblePreprocess OBJECT ${leaksense_counter_default_default_XC8_FILE_TYPE_assemblePreprocess})
    leaksense_counter_default_default_XC8_assemblePreprocess_rule(leaksense_counter_default_default_XC8_assemblePreprocess)
    list(APPEND leaksense_counter_default_library_list "$<TARGET_OBJECTS:leaksense_counter_default_default_XC8_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group default-XC8
if(leaksense_counter_default_default_XC8_FILE_TYPE_compile)
add_library(leaksense_counter_default_default_XC8_compile OBJECT ${leaksense_counter_default_default_XC8_FILE_TYPE_compile})
    leaksense_counter_default_default_XC8_compile_rule(leaksense_counter_default_default_XC8_compile)
    list(APPEND leaksense_counter_default_library_list "$<TARGET_OBJECTS:leaksense_counter_default_default_XC8_compile>")

endif()


# Main target for this project
add_executable(leaksense_counter_default_image_U7B5UFXR ${leaksense_counter_default_library_list})

set_target_properties(leaksense_counter_default_image_U7B5UFXR PROPERTIES
    OUTPUT_NAME "default"
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES "${output_extensions}"
    RUNTIME_OUTPUT_DIRECTORY "${leaksense_counter_default_output_dir}")
target_link_libraries(leaksense_counter_default_image_U7B5UFXR PRIVATE ${leaksense_counter_default_default_XC8_FILE_TYPE_link})

# Add the link options from the rule file.
leaksense_counter_default_link_rule( leaksense_counter_default_image_U7B5UFXR)


