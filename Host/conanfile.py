from conan import ConanFile
from conan.tools.cmake import cmake_layout

class Dependency(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain", "ROSEnv"

    def requirements(self):
        self.requires("serial/1.2.1")

    def layout(self):
        pass
        # cmake_layout(self)