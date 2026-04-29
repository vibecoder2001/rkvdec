add_executable(rkmpp_iommu_fault main.cpp)
target_include_directories(rkmpp_iommu_fault PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
target_link_libraries(rkmpp_iommu_fault PRIVATE setupapi cfgmgr32)
