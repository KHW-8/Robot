from conan import ConanFile
from conan.tools.cmake import CMakeToolchain

class Dependency(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain", "ROSEnv"

    default_options = {
        "*:shared": True,
    }

    def requirements(self):
        self.requires("boost/1.91.0")
        self.requires("gtest/1.17.0")
        self.requires("opencv/4.14.0")
        self.requires("qt/6.11.1")
        self.requires("symengine/0.14.0")
        self.requires("xkbcommon/1.6.0", override=True)
