cmake_minimum_required(VERSION 3.8)
project(codes)  # Nome do projeto deve ser 'codes'

# Verifique se o sistema suporta C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enforce C++17 standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependências do ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Inclua o diretório de cabeçalhos
include_directories(
  include
)

# Adicione os arquivos do componente
add_library(codes_component SHARED
  src/calculadoranode.cpp
)

# Registre o componente
ament_target_dependencies(codes_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
)

# Registre o nó como um componente do ROS 2
rclcpp_components_register_nodes(codes_component "calculadora::MainNode")

# Crie o executável para rodar o nó sem o container
add_executable(codes_main
  src/calculadoramain.cpp
)

ament_target_dependencies(codes_main
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
)

# Vincule o executável ao componente
target_link_libraries(codes_main
  codes_component
)

# Instale o componente
install(TARGETS
  codes_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instale o executável
install(TARGETS
  codes_main
  DESTINATION libexec/${PROJECT_NAME}  # Alterado para 'libexec'
)

# Instale arquivos de launch e parâmetros
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

# Instale os arquivos de include
install(DIRECTORY include/
  DESTINATION include/
)

# Exportar informações do pacote
ament_package()
