set(IMPEDANCETCTRLCOMP_NAME CustomImpedanceTaskControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE IMPEDANCETCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${IMPEDANCETCTRLCOMP_NAME} MODULE
			${IMPEDANCETCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${IMPEDANCETCTRLCOMP_NAME} ${LIBS})

set_target_properties(${IMPEDANCETCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${IMPEDANCETCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${IMPEDANCETCTRLCOMP_NAME} DESTINATION ./PluginComponents)