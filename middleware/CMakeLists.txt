# --- middleware: the seam where this project meets Arena's eCAL transport ---

add_library(camera_tracking_middleware INTERFACE)
target_include_directories(camera_tracking_middleware INTERFACE include)
target_link_libraries(camera_tracking_middleware
    INTERFACE Arena::TransportEcal protobuf::libprotobuf)
