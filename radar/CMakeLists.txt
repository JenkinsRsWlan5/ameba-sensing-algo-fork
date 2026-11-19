if(NOT CONFIG_WIFI_RADAR_ENABLE OR CONFIG_MP_SHRINK)
    return()
endif()
if(NOT (CONFIG_WHC_NONE OR CONFIG_WHC_HOST OR CONFIG_WHC_WPA_SUPPLICANT_OFFLOAD))
    return()
endif()

##########################################################################################
## * This part defines public part of the component
## * Public part will be used as global build configures for all component

set(public_includes)                #public include directories, NOTE: relative path is OK
set(public_definitions)             #public definitions
set(public_libraries)               #public libraries(files), NOTE: linked with whole-archive options

#----------------------------------------#
# Component public part, user config begin

#only the public API header is exported, radar internals stay private
ameba_list_append(public_includes
    ${c_CMPT_SENSING_ALGO_DIR}/include
)

# Component public part, user config end
#----------------------------------------#

#WARNING: Fixed section, DO NOT change!
ameba_global_include(${public_includes})
ameba_global_define(${public_definitions})
ameba_global_library(${public_libraries}) #default: whole-archived

##########################################################################################
## * This part defines private part of the component

set(private_sources)                 #private source files, NOTE: relative path is OK
set(private_includes)                #private include directories, NOTE: relative path is OK
set(private_definitions)             #private definitions
set(private_compile_options)         #private compile_options

#------------------------------#
# Component private part, user config begin

#each scene owns its NN model and the code that differs between scenes
if(CONFIG_RADAR_M_FRIDGE_SCENE)
    set(scene_dir m_fridge)
else()
    set(scene_dir general)
endif()

ameba_list_append(private_sources
    wifi_radar.c
    ${scene_dir}/wifi_radar_${scene_dir}.c
)
ameba_list_append(private_includes ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${scene_dir})
ameba_list_append_if(CONFIG_RADAR_M_FRIDGE_SCENE private_sources m_fridge/radar_fridge_light.c)
if(CONFIG_TFLITE_MICRO_EN)
    ameba_list_append(private_sources
        ${scene_dir}/nn_interface.cc
        ${scene_dir}/models/radar_int8.c
    )
    ameba_list_append(private_definitions TF_LITE_STATIC_MEMORY)
    ameba_list_append(private_compile_options -Wno-unused-parameter -Wno-undef)
endif()

# Component private part, user config end
#------------------------------#

ameba_add_internal_library(wifi_radar
    p_SOURCES
        ${private_sources}
    p_INCLUDES
        ${private_includes}
    p_DEFINITIONS
        ${private_definitions}
    p_COMPILE_OPTIONS
        ${private_compile_options}
)
