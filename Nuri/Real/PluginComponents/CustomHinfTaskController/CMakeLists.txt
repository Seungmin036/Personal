set(CTTCTRLCOMP_NAME CustomHinfTaskControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE CTTCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${CTTCTRLCOMP_NAME} MODULE
			${CTTCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${CTTCTRLCOMP_NAME} ${LIBS})

set_target_properties(${CTTCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${CTTCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${CTTCTRLCOMP_NAME} DESTINATION ./PluginComponents)