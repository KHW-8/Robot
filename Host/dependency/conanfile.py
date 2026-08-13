from conan import ConanFile
from conan.tools.cmake import cmake_layout
from conan.tools.files import copy
import os

class Dependency(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain", "ROSEnv"

    default_options = {
        "*:shared": True,
    }

    def requirements(self):
        self.requires("serial/1.2.1")

    def generate(self):
        # Copy dependencies to build folder
        asset_folder = os.path.join(self.build_folder, "asset")

        for dep in self.dependencies.values():
            # Copy headers
            for includedir in dep.cpp_info.includedirs:
                copy(self, 
                    pattern="*", 
                    src=includedir, 
                    dst=os.path.join(asset_folder, "include", dep.ref.name, str(dep.ref.version))
                )
                
            # Copy sources
            for srcdir in dep.cpp_info.srcdirs:
                copy(self, 
                    pattern="*", 
                    src=srcdir, 
                    dst=os.path.join(asset_folder, "src", dep.ref.name, str(dep.ref.version))
                )