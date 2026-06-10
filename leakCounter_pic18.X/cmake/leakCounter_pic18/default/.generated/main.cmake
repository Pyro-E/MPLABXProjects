include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(leakCounter_pic18_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(leakCounter_pic18_default_default_XC8_FILE_TYPE_assemble)
add_library(leakCounter_pic18_default_default_XC8_assemble OBJECT ${leakCounter_pic18_default_default_XC8_FILE_TYPE_assemble})
    leakCounter_pic18_default_default_XC8_assemble_rule(leakCounter_pic18_default_default_XC8_assemble)
    list(APPEND leakCounter_pic18_default_library_list "$<TARGET_OBJECTS:leakCounter_pic18_default_default_XC8_assemble>")

endif()

# Handle files with suffix S, for group default-XC8
if(leakCounter_pic18_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(leakCounter_pic18_default_default_XC8_assemblePreprocess OBJECT ${leakCounter_pic18_default_default_XC8_FILE_TYPE_assemblePreprocess})
    leakCounter_pic18_default_default_XC8_assemblePreprocess_rule(leakCounter_pic18_default_default_XC8_assemblePreprocess)
    list(APPEND leakCounter_pic18_default_library_list "$<TARGET_OBJECTS:leakCounter_pic18_default_default_XC8_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group default-XC8
if(leakCounter_pic18_default_default_XC8_FILE_TYPE_compile)
add_library(leakCounter_pic18_default_default_XC8_compile OBJECT ${leakCounter_pic18_default_default_XC8_FILE_TYPE_compile})
    leakCounter_pic18_default_default_XC8_compile_rule(leakCounter_pic18_default_default_XC8_compile)
    list(APPEND leakCounter_pic18_default_library_list "$<TARGET_OBJECTS:leakCounter_pic18_default_default_XC8_compile>")

endif()


# Main target for this project
add_executable(leakCounter_pic18_default_image_YH9KO_2D ${leakCounter_pic18_default_library_list})

set_target_properties(leakCounter_pic18_default_image_YH9KO_2D PROPERTIES
    OUTPUT_NAME "default"
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES "${output_extensions}"
    RUNTIME_OUTPUT_DIRECTORY "${leakCounter_pic18_default_output_dir}")
target_link_libraries(leakCounter_pic18_default_image_YH9KO_2D PRIVATE ${leakCounter_pic18_default_default_XC8_FILE_TYPE_link})

# Add the link options from the rule file.
leakCounter_pic18_default_link_rule( leakCounter_pic18_default_image_YH9KO_2D)


