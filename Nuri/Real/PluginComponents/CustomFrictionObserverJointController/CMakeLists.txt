set(CFOJCTRLCOMP_NAME CustomFrictionObserverJointControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE CFOJCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${CFOJCTRLCOMP_NAME} MODULE
			${CFOJCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${CFOJCTRLCOMP_NAME} ${LIBS})

set_target_properties(${CFOJCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${CFOJCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${CFOJCTRLCOMP_NAME} DESTINATION ./PluginComponents)