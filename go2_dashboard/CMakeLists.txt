cmake_minimum_required(VERSION 3.8)
project(go2_dashboard)

find_package(ament_cmake REQUIRED)

set(WEB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/web)
set(WEB_DIST ${WEB_DIR}/dist)

find_program(NPM_EXECUTABLE npm)
if(NPM_EXECUTABLE AND EXISTS ${WEB_DIR}/package.json)
  message(STATUS "go2_dashboard: building web UI with npm")
  execute_process(
    COMMAND ${NPM_EXECUTABLE} ci
    WORKING_DIRECTORY ${WEB_DIR}
    RESULT_VARIABLE NPM_CI_RESULT
  )
  if(NOT NPM_CI_RESULT EQUAL 0)
    message(WARNING "go2_dashboard: npm ci failed; run manually in web/")
  else()
    execute_process(
      COMMAND ${NPM_EXECUTABLE} run build
      WORKING_DIRECTORY ${WEB_DIR}
      RESULT_VARIABLE NPM_BUILD_RESULT
    )
    if(NOT NPM_BUILD_RESULT EQUAL 0)
      message(WARNING "go2_dashboard: npm run build failed; run manually in web/")
    endif()
  endif()
elseif(EXISTS ${WEB_DIR}/package.json)
  message(WARNING "go2_dashboard: npm not found; build web UI manually: cd web && npm ci && npm run build")
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS ${WEB_DIST})
  install(DIRECTORY ${WEB_DIST}/
    DESTINATION share/${PROJECT_NAME}/web
  )
else()
  message(WARNING "go2_dashboard: web/dist missing; dashboard will not be served until you build the web UI")
endif()

ament_package()
