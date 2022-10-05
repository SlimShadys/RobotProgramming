add_library(core_library SHARED
  utils/blob.cpp utils/deserializer.cpp utils/id_context.cpp
  utils/id_placeholder.cpp utils/identifiable.cpp utils/object_data.cpp
  utils/property.cpp utils/property_container.cpp utils/property_container_manager.cpp
  utils/property_eigen.cpp utils/property_identifiable.cpp utils/serializable.cpp
  utils/serialization_context.cpp utils/serializer.cpp
  
  messages/base_sensor_message.cpp messages/odometry_message.cpp
  messages/pose_message.cpp
  
  data_structures/matrix.hpp
)
