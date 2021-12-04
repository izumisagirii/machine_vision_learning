set(DAHUA_INCLUDE_DIR /opt/MVS/include)
set(LIB_NAMES FormatConversion libFormatConversion.so
        libGCBase_gcc48_v3_0.so
        libGenApi_gcc48_v3_0.so
        liblog4cpp_gcc48_v3_0.so
        libLog_gcc48_v3_0.so
        libMathParser_gcc48_v3_0.so
        libMediaProcess.so
        libMvCameraControl.so
        libMvCameraControl.so.3.1.0.3
        libMVGigEVisionSDK.so
        libMVGigEVisionSDK.so.3.1.0.3
        libMVRender.so
        libMvUsb3vTL.so
        libMvUsb3vTL.so.3.1.0.3
        libNodeMapData_gcc48_v3_0.so
        libXmlParser_gcc48_v3_0.so)
set(PATH_NAMES /opt/MVS/lib/aarch64 /opt/MVS/lib/aarch64 /opt/MVS/lib)
FOREACH (LIB_NAME ${LIB_NAMES})
    FIND_LIBRARY(MVSCAM_LIBRARY NAMES ${LIB_NAME} PATHS ${PATH_NAMES})
    IF (MVSCAM_LIBRARY)
        SET(MVSCAM_LIBRARIES ${MVSCAM_LIBRARIES} ${MVSCAM_LIBRARY})
        IF (NOT MVSCAM_FIND_QUIETLY)
            MESSAGE(STATUS "MVSCAM Found: ${MVSCAM_LIBRARY}")
        ENDIF (NOT MVSCAM_FIND_QUIETLY)
    ELSE (MVSCAM_LIBRARY)
        IF (MVSCAM_FIND_REQUIRED)
            MESSAGE(FATAL_ERROR "Could not found MVSCAM ${MVSCAM_LIBRARY}")
        ENDIF (MVSCAM_FIND_REQUIRED)
    ENDIF (MVSCAM_LIBRARY)
    UNSET(MVSCAM_LIBRARY CACHE)
ENDFOREACH (LIB_NAME)

