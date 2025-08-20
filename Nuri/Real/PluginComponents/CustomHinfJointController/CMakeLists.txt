set(HINFJCTRLCOMP_NAME CustomHinfJointControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE HINFJCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${HINFJCTRLCOMP_NAME} MODULE
			${HINFJCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${HINFJCTRLCOMP_NAME} ${LIBS})

set_target_properties(${HINFJCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${HINFJCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${HINFJCTRLCOMP_NAME} DESTINATION ./PluginComponents)