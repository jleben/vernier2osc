cmake_minimum_required(VERSION 2.8)

project(vernier2osc)

set( src
main.cpp
)

add_executable(vernier2osc ${src})

if(WIN32)
    include_directories( ${CMAKE_SOURCE_DIR}/NGIO/include )
    include_directories( "C:/Users/Jakob/src/liblo-0.26" )

    set( NGIO_LIBRARY  ${CMAKE_SOURCE_DIR}/NGIO/lib/NGIO_lib.dll )
    set( LO_LIBRARY "C:/Users/Jakob/src/liblo-0.26/src/.libs/liblo.a" )
    target_link_libraries(vernier2osc
        ${NGIO_LIBRARY}
        ${LO_LIBRARY}
        ws2_32
    )

    install( TARGETS vernier2osc DESTINATION vernier2osc )

    set( DEPENDENCY_DIRS
         ${CMAKE_SOURCE_DIR}/NGIO/lib/
         "C:/Users/Jakob/src/liblo-0.26/src/.libs/"
    )

    install( CODE "
        include(BundleUtilities)
        fixup_bundle(
            ${CMAKE_INSTALL_PREFIX}/vernier2osc/vernier2osc.exe
            \"\"
            ${DEPENDENCY_DIRS}
        )
    ")

elseif(UNIX)

    include_directories( /usr/include/NGIO-1.0.0 )
    include_directories( /usr/include/lo )
    target_link_libraries(vernier2osc NGIO usb-1.0 lo)
    install( TARGETS vernier2osc DESTINATION bin )

endif()
