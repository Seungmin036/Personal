set(PDJCTRLCOMP_NAME CustomPdJointControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE PDJCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${PDJCTRLCOMP_NAME} MODULE
			${PDJCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${PDJCTRLCOMP_NAME} ${LIBS})

set_target_properties(${PDJCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${PDJCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${PDJCTRLCOMP_NAME} DESTINATION ./PluginComponents)