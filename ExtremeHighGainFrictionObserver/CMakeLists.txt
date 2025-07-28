set(HGFOCOMP_NAME ExtremeHighGainFrictionObserverCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE HGFOCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${HGFOCOMP_NAME} MODULE
			${HGFOCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${HGFOCOMP_NAME} ${LIBS})

set_target_properties(${HGFOCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${HGFOCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${HGFOCOMP_NAME} DESTINATION ./PluginComponents)