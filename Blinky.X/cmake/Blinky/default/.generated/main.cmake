include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(Blinky_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(Blinky_default_default_XC8_FILE_TYPE_assemble)
add_library(Blinky_default_default_XC8_assemble OBJECT ${Blinky_default_default_XC8_FILE_TYPE_assemble})
    Blinky_default_default_XC8_assemble_rule(Blinky_default_default_XC8_assemble)
    list(APPEND Blinky_default_library_list "$<TARGET_OBJECTS:Blinky_default_default_XC8_assemble>")

endif()

# Handle files with suffix S, for group default-XC8
if(Blinky_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(Blinky_default_default_XC8_assemblePreprocess OBJECT ${Blinky_default_default_XC8_FILE_TYPE_assemblePreprocess})
    Blinky_default_default_XC8_assemblePreprocess_rule(Blinky_default_default_XC8_assemblePreprocess)
    list(APPEND Blinky_default_library_list "$<TARGET_OBJECTS:Blinky_default_default_XC8_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group default-XC8
if(Blinky_default_default_XC8_FILE_TYPE_compile)
add_library(Blinky_default_default_XC8_compile OBJECT ${Blinky_default_default_XC8_FILE_TYPE_compile})
    Blinky_default_default_XC8_compile_rule(Blinky_default_default_XC8_compile)
    list(APPEND Blinky_default_library_list "$<TARGET_OBJECTS:Blinky_default_default_XC8_compile>")

endif()


# Main target for this project
add_executable(Blinky_default_image_fpF8sdqc ${Blinky_default_library_list})

set_target_properties(Blinky_default_image_fpF8sdqc PROPERTIES
    OUTPUT_NAME "default"
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES "${output_extensions}"
    RUNTIME_OUTPUT_DIRECTORY "${Blinky_default_output_dir}")
target_link_libraries(Blinky_default_image_fpF8sdqc PRIVATE ${Blinky_default_default_XC8_FILE_TYPE_link})

# Add the link options from the rule file.
Blinky_default_link_rule( Blinky_default_image_fpF8sdqc)


