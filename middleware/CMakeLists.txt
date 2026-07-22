# --- middleware: thin eCAL wrappers shared by backend + frontend ---
# Migrated from mulib's mu_middleware (external/mulib/libs/mu_middleware) so
# eCAL pub/sub no longer depends on mulib. Only the pieces this project
# actually uses were carried over: the typed protobuf topic wrappers and the
# ref-counted eCAL::Initialize/Finalize lifecycle they share.
add_library(camera_tracking_middleware STATIC
    src/EcalProtoTopic.cpp
)
target_include_directories(camera_tracking_middleware PUBLIC include)
# PUBLIC: the header instantiates eCAL::protobuf::CPublisher/CSubscriber<T>
# in consumer TUs, so consumers need eCAL + protobuf usage requirements too.
target_link_libraries(camera_tracking_middleware PUBLIC eCAL::core protobuf::libprotobuf)
