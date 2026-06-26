cmake_minimum_required(VERSION 3.8)
project(l6_rl_training)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/l6_train_ppo
  scripts/l6_eval_policy
  scripts/l6_visualize_policy
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
